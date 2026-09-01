#!/usr/bin/env python3
"""
pick_and_sort_node.py

Consumes JSON color detections from color_detector_node and drives MoveIt2
(via pymoveit2) to pick a specific colored cube and drop it into a single
basket.

Two ways to trigger a pick:
  - ~/pick_color (franka_color_sort_interfaces/srv/PickColor): pick exactly
    ONE cube of the requested color and place it in the basket. Request
    field: string color ("red" | "green" | "blue"). Blocks until the motion
    completes and returns success/message.
  - ~/start_sorting (std_srvs/Trigger): original auto-sort-everything loop,
    kept for convenience/testing. Picks whatever cube is detected first,
    repeatedly, until none remain. ~/stop_sorting cancels it.

Calibrated constants (from debugging session):
  - table_z_m = 0.24 -> grasp depth target = table_z - grasp_depth_offset
    = 0.22, matching the cube's real physics-settled center height
    (table top at z=0.2 + half cube height 0.02).
  - basket_*_m -> single basket placed on the table at (0.6, -0.25),
    safely inside the table's 0.8x0.8 footprint (spans x:[0.2,1.0],
    y:[-0.4,0.4]) so cubes don't fall off the edge like the earlier
    place_y=-0.4 attempt did.

>>> THINGS YOU MUST VERIFY / ADJUST FOR YOUR SETUP <<<
  1. EE_LINK = "fer_hand_tcp" — confirmed correct via TF echo during
     debugging (matches DOWNWARD_ORIENTATION exactly).
  2. If you resize/move the basket in the world SDF, update
     basket_x_m/basket_y_m/basket_z_m parameters to match.
"""

import json
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from pymoveit2 import MoveIt2

from franka_color_sort_interfaces.srv import PickColor


PLANNING_GROUP = "fer_arm"
BASE_LINK = "fer_link0"
EE_LINK = "fer_hand_tcp"
ARM_JOINT_NAMES = [
    "fer_joint1", "fer_joint2", "fer_joint3",
    "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7",
]

GRIPPER_ACTION_NAME = "/fer_hand_controller/follow_joint_trajectory"
GRIPPER_JOINT_NAME = "fer_finger_joint1"
GRIPPER_OPEN_POS_DEFAULT = 0.035      # from fer.srdf group_state "open"
GRIPPER_CLOSED_POS_DEFAULT = 0.0      # from fer.srdf group_state "closed"
GRIPPER_MOVE_DURATION_S = 1.0
GRIPPER_SETTLE_DELAY_S = 0.3          # let fingers physically settle before retreating

VALID_COLORS = {"red", "green", "blue"}

# Gripper "pointing straight down" quaternion for the fer_hand_tcp frame.
DOWNWARD_ORIENTATION = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


@dataclass
class Detection:
    color: str
    pixel_x: float
    pixel_y: float
    world_x: float
    world_y: float
    area: float


class PickAndSortNode(Node):
    def __init__(self):
        super().__init__("pick_and_sort_node")

        # ---- Parameters ----
        self.declare_parameter("detections_topic", "/color_detector/detections")
        self.declare_parameter("base_frame", BASE_LINK)
        self.declare_parameter("table_z_m", 0.24)
        self.declare_parameter("grasp_depth_offset_m", 0.02)
        self.declare_parameter("approach_height_m", 0.15)
        self.declare_parameter("gripper_open_pos", GRIPPER_OPEN_POS_DEFAULT)
        self.declare_parameter("gripper_closed_pos", GRIPPER_CLOSED_POS_DEFAULT)
        self.declare_parameter("basket_x_m", 0.6)
        self.declare_parameter("basket_y_m", -0.25)
        self.declare_parameter("basket_z_m", 0.22)
        self.declare_parameter("min_cube_area", 300)
        self.declare_parameter("max_pick_attempts", 3)
        self.declare_parameter("settle_delay_s", 0.5)

        p = self.get_parameter
        self.base_frame = p("base_frame").value
        self.table_z = p("table_z_m").value
        self.grasp_depth_offset = p("grasp_depth_offset_m").value
        self.approach_height = p("approach_height_m").value
        self.gripper_open_pos = p("gripper_open_pos").value
        self.gripper_closed_pos = p("gripper_closed_pos").value
        self.place_pose_xyz = (p("basket_x_m").value, p("basket_y_m").value, p("basket_z_m").value)
        self.min_cube_area = p("min_cube_area").value
        self.max_pick_attempts = p("max_pick_attempts").value
        self.settle_delay = p("settle_delay_s").value

        # ---- State ----
        self._lock = threading.Lock()
        self.latest_detections: list[Detection] = []
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_requested = False
        self._busy_lock = threading.Lock()  # guards against overlapping pick_color/start_sorting runs

        cb_group = ReentrantCallbackGroup()

        # ---- ROS interfaces ----
        self.create_subscription(
            String,
            p("detections_topic").value,
            self._detections_callback,
            10,
            callback_group=cb_group,
        )

        self.start_srv = self.create_service(
            Trigger, "~/start_sorting", self._start_sorting_callback, callback_group=cb_group
        )
        self.stop_srv = self.create_service(
            Trigger, "~/stop_sorting", self._stop_sorting_callback, callback_group=cb_group
        )
        self.pick_color_srv = self.create_service(
            PickColor, "~/pick_color", self._pick_color_callback, callback_group=cb_group
        )

        self.gripper_client = ActionClient(self, FollowJointTrajectory, GRIPPER_ACTION_NAME)

        # ---- MoveIt2 (pymoveit2) ----
        self.get_logger().info("Initializing MoveIt2 (pymoveit2)...")
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=EE_LINK,
            group_name=PLANNING_GROUP,
            callback_group=cb_group,
        )
        self.moveit2.allowed_planning_time = 5.0
        self.moveit2.num_planning_attempts = 10
        self.moveit2.pipeline_id = "ompl"
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.get_logger().info("MoveIt2 ready.")

        # Move to a safe home pose to clear the fer_hand/fer_link7
        # self-collision present at the default spawn configuration.
        self.get_logger().info("Moving to safe home configuration...")
        self.moveit2.move_to_configuration(
            joint_positions=[0.0, -0.5, 0.0, -2.0, 0.0, 1.8, 0.785398],
            joint_names=ARM_JOINT_NAMES,
        )
        self.moveit2.wait_until_executed()
        self.get_logger().info("Home configuration reached.")

    # ------------------------------------------------------------------
    # Subscription callback — cheap, never blocks
    # ------------------------------------------------------------------
    def _detections_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Bad detections JSON: {exc}")
            return

        detections = []
        for d in payload.get("detections", []):
            if d.get("area", 0) < self.min_cube_area:
                continue
            try:
                detections.append(Detection(
                    color=d["color"],
                    pixel_x=d["pixel_x"],
                    pixel_y=d["pixel_y"],
                    world_x=d["world_x"],
                    world_y=d["world_y"],
                    area=d["area"],
                ))
            except KeyError as exc:
                self.get_logger().warn(f"Detection missing field {exc}, skipping")

        with self._lock:
            self.latest_detections = detections

    def _get_snapshot(self) -> list[Detection]:
        with self._lock:
            return list(self.latest_detections)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _pick_color_callback(self, request, response):
        color = (request.color or "").strip().lower()
        if color not in VALID_COLORS:
            response.success = False
            response.message = f"Invalid color '{request.color}'. Must be one of {sorted(VALID_COLORS)}."
            return response

        if not self._busy_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another pick/sort operation is already in progress."
            return response

        try:
            snapshot = self._get_snapshot()
            candidates = [d for d in snapshot if d.color == color]
            if not candidates:
                response.success = False
                response.message = f"No {color} cube currently detected."
                return response

            target = max(candidates, key=lambda d: d.area)  # largest/most confident match
            self.get_logger().info(
                f"[pick_color] Picking {color} cube at "
                f"({target.world_x:.3f}, {target.world_y:.3f})"
            )

            success = self._pick_and_place(target)
            if success:
                response.success = True
                response.message = f"Picked {color} cube and placed it in the basket."
            else:
                response.success = False
                response.message = f"Failed to pick {color} cube after {self.max_pick_attempts} attempts."
            return response
        finally:
            self._busy_lock.release()

    def _start_sorting_callback(self, request, response):
        if self._worker_thread is not None and self._worker_thread.is_alive():
            response.success = False
            response.message = "Sorting already in progress."
            return response

        if not self._busy_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another pick/sort operation is already in progress."
            return response

        self._stop_requested = False
        self._worker_thread = threading.Thread(target=self._sorting_loop, daemon=True)
        self._worker_thread.start()
        response.success = True
        response.message = "Sorting started."
        return response

    def _stop_sorting_callback(self, request, response):
        self._stop_requested = True
        response.success = True
        response.message = "Stop requested; will halt after current cube."
        return response

    # ------------------------------------------------------------------
    # Worker thread: auto-sort-everything loop (used by ~/start_sorting)
    # ------------------------------------------------------------------
    def _sorting_loop(self):
        self.get_logger().info("Sorting loop started.")
        consecutive_empty = 0

        try:
            while rclpy.ok() and not self._stop_requested:
                snapshot = self._get_snapshot()

                if not snapshot:
                    consecutive_empty += 1
                    if consecutive_empty >= 3:
                        self.get_logger().info("No cubes detected. Sorting complete.")
                        break
                    time.sleep(self.settle_delay)
                    continue

                consecutive_empty = 0
                target = snapshot[0]

                self.get_logger().info(
                    f"Picking {target.color} cube at "
                    f"({target.world_x:.3f}, {target.world_y:.3f})"
                )

                success = self._pick_and_place(target)
                if not success:
                    self.get_logger().warn(
                        f"Failed to pick {target.color} cube after retries; skipping this cycle."
                    )

                time.sleep(self.settle_delay)

            self.get_logger().info("Sorting loop exited.")
        finally:
            self._busy_lock.release()

    # ------------------------------------------------------------------
    # Shared pick-and-place routine (used by both pick_color and sorting loop)
    # ------------------------------------------------------------------
    def _pick_and_place(self, target: Detection) -> bool:
        for attempt in range(1, self.max_pick_attempts + 1):
            self.get_logger().info(f"Attempt {attempt}/{self.max_pick_attempts}")

            fresh = self._closest_match(target)
            if fresh is None:
                self.get_logger().warn("Target cube no longer detected; aborting this attempt.")
                return False

            if not self._move_to_pose(fresh.world_x, fresh.world_y,
                                       self.table_z + self.approach_height,
                                       DOWNWARD_ORIENTATION):
                continue
            if not self._set_gripper(self.gripper_open_pos):
                continue
            if not self._move_to_pose(fresh.world_x, fresh.world_y,
                                       self.table_z - self.grasp_depth_offset,
                                       DOWNWARD_ORIENTATION):
                continue
            if not self._set_gripper(self.gripper_closed_pos):
                continue
            time.sleep(GRIPPER_SETTLE_DELAY_S)  # let fingers actually grip before lifting
            if not self._move_to_pose(fresh.world_x, fresh.world_y,
                                       self.table_z + self.approach_height,
                                       DOWNWARD_ORIENTATION):
                continue

            # Place in basket
            bx, by, bz = self.place_pose_xyz
            if not self._move_to_pose(bx, by, bz + self.approach_height, DOWNWARD_ORIENTATION):
                continue
            if not self._move_to_pose(bx, by, bz, DOWNWARD_ORIENTATION):
                continue
            if not self._set_gripper(self.gripper_open_pos):
                continue
            if not self._move_to_pose(bx, by, bz + self.approach_height, DOWNWARD_ORIENTATION):
                continue

            return True

        return False

    def _closest_match(self, target: Detection) -> Optional[Detection]:
        snapshot = self._get_snapshot()
        candidates = [d for d in snapshot if d.color == target.color]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda d: (d.world_x - target.world_x) ** 2 + (d.world_y - target.world_y) ** 2,
        )

    # ------------------------------------------------------------------
    # MoveIt2 motion helper (pymoveit2)
    # ------------------------------------------------------------------
    def _move_to_pose(self, x: float, y: float, z: float, orientation: Quaternion) -> bool:
        quat_xyzw = [orientation.x, orientation.y, orientation.z, orientation.w]

        self.moveit2.pipeline_id = "ompl"
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=quat_xyzw,
            cartesian=False,
        )
        success = self.moveit2.wait_until_executed()

        if not success:
            self.get_logger().warn(
                f"Move to ({x:.3f}, {y:.3f}, {z:.3f}) failed (plan or exec)."
            )
        return success

    # ------------------------------------------------------------------
    # Gripper control — direct FollowJointTrajectory action client
    # (fer_hand_controller does not expose a GripperCommand action)
    # ------------------------------------------------------------------
    def _set_gripper(self, position: float) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT_NAME]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(seconds=GRIPPER_MOVE_DURATION_S).to_msg()
        goal.trajectory.points = [point]

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        result = result_future.result()
        return result is not None


def main(args=None):
    rclpy.init(args=args)
    node = PickAndSortNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

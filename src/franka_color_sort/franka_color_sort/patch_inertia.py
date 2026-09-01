#!/usr/bin/env python3
"""
patch_inertia.py

Reads a processed URDF from stdin, checks every <inertial><inertia .../>
block for the triangle inequality (largest principal moment <= sum of the
other two), which Ignition enforces and franka_description's FER inertia
data currently violates for at least one link (fer_link4).

For any violating tensor, adds the minimum isotropic correction (the same
epsilon added to ixx, iyy, izz only) needed to satisfy the inequality, plus
a small safety margin. Adding epsilon*Identity shifts all three eigenvalues
by the same amount without rotating the tensor's principal axes, so this is
the smallest-impact fix available -- mass, origin, and off-diagonal terms
are left untouched.

Usage: xacro <file.xacro> <args...> | patch_inertia > patched.urdf
"""

import sys
import xml.etree.ElementTree as ET

import numpy as np

MARGIN = 2e-4  # extra safety margin added beyond the exact violation


def fix_inertia_element(inertia_elem):
    ixx = float(inertia_elem.get('ixx'))
    iyy = float(inertia_elem.get('iyy'))
    izz = float(inertia_elem.get('izz'))
    ixy = float(inertia_elem.get('ixy', '0'))
    ixz = float(inertia_elem.get('ixz', '0'))
    iyz = float(inertia_elem.get('iyz', '0'))

    matrix = np.array([
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ])
    eigvals = sorted(np.linalg.eigvalsh(matrix))
    lam_min, lam_mid, lam_max = eigvals

    violation = lam_max - (lam_mid + lam_min)
    if violation <= 0:
        return False  # already valid, nothing to do

    epsilon = violation + MARGIN
    inertia_elem.set('ixx', repr(ixx + epsilon))
    inertia_elem.set('iyy', repr(iyy + epsilon))
    inertia_elem.set('izz', repr(izz + epsilon))
    return True


def main():
    urdf_text = sys.stdin.read()
    root = ET.fromstring(urdf_text)

    fixed_links = []
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        inertia = inertial.find('inertia')
        if inertia is None:
            continue
        if fix_inertia_element(inertia):
            fixed_links.append(link.get('name'))

    if fixed_links:
        print(f"patch_inertia: corrected invalid inertia on: {fixed_links}",
              file=sys.stderr)

    sys.stdout.write(ET.tostring(root, encoding='unicode'))


if __name__ == '__main__':
    main()

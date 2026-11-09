#!/usr/bin/env python3
"""Print the BRL_lab planning model: the sim summit model (summit_ur5e_moveit_config
mobile_manipulator.urdf/.srdf, as used by brl_hqp_sim.launch) adapted to the lab UR5e.

  urdf: arm joints renamed ur5e_X_joint -> X_joint (lab /joint_states and HQP names; links
        keep ur5e_ names), gripper/wheel joints fixed (no states in the lab; gripper open),
        arm limits = real UR5e +-360 deg (sim model: +-180), elbow +-elbow_limit_deg,
        robot_base_footprint (BRL target frame) = summit_xl_base_footprint (base is static).
  srdf: same renames; drops disable_collisions entries that hid real lab collisions:
        non-adjacent arm pairs (e.g. upper_arm/wrist_1 labelled "Adjacent") and gripper or
        bracket vs arm links other than the rigid mount (wrist_3/ee_link/tool0).
Everything else (summit body, gripper, groups, named states, other pairs) is unchanged.

usage: make_brl_lab_summit.py urdf|srdf [--elbow-limit-deg 160]
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import rospkg

ARM = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
RENAME = {'ur5e_%s_joint' % name: '%s_joint' % name for name in ARM}
ARM_LINKS = {'ur5e_base_link', 'ur5e_shoulder_link', 'ur5e_upper_arm_link', 'ur5e_forearm_link',
             'ur5e_wrist_1_link', 'ur5e_wrist_2_link', 'ur5e_wrist_3_link', 'ur5e_ee_link', 'ur5e_tool0'}
ARM_ALLOWED = {frozenset(pair) for pair in [
    ('ur5e_base_link', 'ur5e_shoulder_link'), ('ur5e_shoulder_link', 'ur5e_upper_arm_link'),
    ('ur5e_upper_arm_link', 'ur5e_forearm_link'), ('ur5e_forearm_link', 'ur5e_wrist_1_link'),
    ('ur5e_wrist_1_link', 'ur5e_wrist_2_link'), ('ur5e_wrist_2_link', 'ur5e_wrist_3_link'),
    ('ur5e_wrist_1_link', 'ur5e_wrist_3_link'), ('ur5e_wrist_3_link', 'ur5e_ee_link'),
    ('ur5e_wrist_2_link', 'ur5e_ee_link'), ('ur5e_ee_link', 'ur5e_tool0'), ('ur5e_wrist_3_link', 'ur5e_tool0')]}
MOUNT = {'ur5e_wrist_3_link', 'ur5e_ee_link', 'ur5e_tool0'}


def end_effector(link):
    return link.startswith('gripper_') or link in ('camera_bracket', 'graps_link', 'hand_camera_link')


def frozen(joint):
    return joint.startswith('gripper_') or joint.startswith('summit_xl_') and joint.endswith('wheel_joint')


def urdf(path, elbow_limit_deg):
    robot = ET.parse(path).getroot()
    for joint in robot.findall('joint'):
        name = joint.get('name')
        if name in RENAME:
            joint.set('name', RENAME[name])
        if name in RENAME:
            # Real UR5e ranges (the sim model uses +-180 deg, but the lab arm can be anywhere in
            # +-360 deg; a start state outside the model bounds makes every plan time out).
            limit = joint.find('limit')
            bound = math.radians(elbow_limit_deg) if RENAME[name] == 'elbow_joint' else 2 * math.pi
            limit.set('lower', repr(-bound))
            limit.set('upper', repr(bound))
        if frozen(name) and joint.get('type') != 'fixed':
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'mimic', 'dynamics', 'safety_controller'):
                for child in joint.findall(tag):
                    joint.remove(child)
    for mimic in robot.iter('mimic'):
        mimic.set('joint', RENAME.get(mimic.get('joint'), mimic.get('joint')))
    # BRL targets (pick, detections) are given in robot_base_footprint, the BRL robot's base
    # footprint. The lab base never moves, so it is the summit footprint (identity).
    if robot.find("link[@name='robot_base_footprint']") is None:
        ET.SubElement(robot, 'link', name='robot_base_footprint')
        joint = ET.SubElement(robot, 'joint', name='robot_base_footprint_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='summit_xl_base_footprint')
        ET.SubElement(joint, 'child', link='robot_base_footprint')
        ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    return robot


def srdf(path):
    robot = ET.parse(path).getroot()
    for element in robot.iter('joint'):
        element.set('name', RENAME.get(element.get('name'), element.get('name')))
    for passive in list(robot.findall('passive_joint')):
        if frozen(passive.get('name')):
            robot.remove(passive)
    removed = []
    for pair in list(robot.findall('disable_collisions')):
        a, b = pair.get('link1'), pair.get('link2')
        wrong_arm = a in ARM_LINKS and b in ARM_LINKS and frozenset((a, b)) not in ARM_ALLOWED
        ee_vs_arm = (end_effector(a) and b in ARM_LINKS - MOUNT) or (end_effector(b) and a in ARM_LINKS - MOUNT)
        if wrong_arm or ee_vs_arm:
            robot.remove(pair)
            removed.append('%s/%s' % (a, b))
    sys.stderr.write('[make_brl_lab_summit] removed %d disable_collisions: %s\n' % (len(removed), ', '.join(removed)))
    return robot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('kind', choices=['urdf', 'srdf'])
    parser.add_argument('--elbow-limit-deg', type=float, default=160.0)
    args, _ = parser.parse_known_args()  # ignore roslaunch extras such as __name:=
    config = os.path.join(rospkg.RosPack().get_path('summit_ur5e_moveit_config'), 'config')
    if args.kind == 'urdf':
        robot = urdf(os.path.join(config, 'mobile_manipulator.urdf'), args.elbow_limit_deg)
    else:
        robot = srdf(os.path.join(config, 'mobile_manipulator.srdf'))
    sys.stdout.write(ET.tostring(robot, encoding='unicode'))


if __name__ == '__main__':
    main()

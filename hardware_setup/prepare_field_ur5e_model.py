#!/usr/bin/env python3
"""Read the field URDF and export its arm chain only. Sends no robot commands."""
import argparse
import copy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import rospy

NAMES = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
         'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']


def extract(text):
    source = ET.fromstring(text)
    links = {node.get('name'): node for node in source.findall('link')}
    by_child = {node.find('child').get('link'): node for node in source.findall('joint')}
    base, tip = 'ur5e_base_link', 'ur5e_wrist_3_link'
    chain = []
    current = tip
    while current != base:
        if current not in by_child or len(chain) > len(by_child):
            raise ValueError('Cannot find standalone field arm chain')
        joint = by_child[current]
        chain.append(joint)
        current = joint.find('parent').get('link')
    chain.reverse()
    actuated = [node.get('name') for node in chain if node.get('type') != 'fixed']
    if actuated != NAMES:
        raise ValueError('Unexpected field arm joints: ' + repr(actuated))
    selected_links = {base} | {node.find('child').get('link') for node in chain}
    selected_joints = list(chain)
    # Keep fixed tool/gripper attachments, but never actuated fingers or wheels.
    changed = True
    while changed:
        changed = False
        for joint in source.findall('joint'):
            child = joint.find('child').get('link')
            if joint.get('type') == 'fixed' and joint.find('parent').get('link') in selected_links and child not in selected_links:
                selected_links.add(child)
                selected_joints.append(joint)
                changed = True
    result = ET.Element('robot', name='field_ur5e_arm')
    result.append(ET.Comment('Read-only snapshot of field arm kinematics; mobile base excluded.'))
    for name in sorted(selected_links):
        result.append(copy.deepcopy(links[name]))
    for joint in selected_joints:
        result.append(copy.deepcopy(joint))
    return ET.tostring(result, encoding='unicode')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-param', default='/brl_robot/robot_description')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent/'field/ur5e_arm.urdf')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('prepare_field_arm_model', anonymous=True)
    text = rospy.get_param(args.source_param)
    output = extract(text)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output + '\n')
    metadata = dict(source_parameter=args.source_param, source_sha256=hashlib.sha256(text.encode()).hexdigest(),
                    exported_at=datetime.now().isoformat(), joint_names=NAMES,
                    base_link='ur5e_base_link', controlled_link='ur5e_wrist_3_link',
                    note='Matches the published field model; not a factory calibration extraction.')
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print('Saved field arm-only model: ' + str(args.output))
    print('No ROS parameters, controllers or robot commands changed.')


if __name__ == '__main__':
    main()

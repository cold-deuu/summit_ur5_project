#!/usr/bin/env python3
"""Print BRL SRDF with the live URDF's robot name for roslaunch's command parameter."""
import argparse
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import xmlrpc.client


def normalize(urdf, srdf):
    model = ET.fromstring(urdf)
    semantic = ET.fromstring(srdf)
    if model.tag != 'robot' or semantic.tag != 'robot' or not model.get('name'):
        raise RuntimeError('Expected named robot URDF and SRDF documents')
    semantic.set('name', model.get('name'))
    links = {link.get('name') for link in model.findall('link')}
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    groups = {group.get('name') for group in semantic.findall('group')}
    for effector in list(semantic.findall('end_effector')):
        # A tool needs a separate group, not the single manipulator group itself.
        if len(groups) == 1 or effector.get('group') not in groups:
            semantic.remove(effector)
    # Never silently use collision exclusions or planning groups for another model.
    for element in semantic.iter():
        if element.tag == 'disable_collisions':
            required = [element.get('link1'), element.get('link2')]
        elif element.tag == 'chain':
            required = [element.get('base_link'), element.get('tip_link')]
        elif element.tag == 'link':
            required = [element.get('name')]
        else:
            required = []
        for name in required:
            if name not in links:
                raise RuntimeError('SRDF link missing from live URDF: '+str(name))
        if element.tag in ('joint', 'passive_joint') and element.get('name') not in joints:
            raise RuntimeError('SRDF joint missing from live URDF: '+str(element.get('name')))
    manipulator = next((group for group in semantic.findall('group') if group.get('name') == 'manipulator'), None)
    if manipulator is None or len(manipulator.findall('chain')) != 1:
        raise RuntimeError('BRL SRDF must define one manipulator chain')
    chain = manipulator.find('chain')
    by_child = {joint.find('child').get('link'): joint for joint in joints.values()}
    tip = chain.get('tip_link'); visited = set(); actuated = []
    while tip != chain.get('base_link'):
        if tip in visited or tip not in by_child:
            raise RuntimeError('SRDF manipulator chain is disconnected in live URDF')
        visited.add(tip); joint = by_child[tip]
        if joint.get('type') != 'fixed': actuated.append(joint.get('name'))
        tip = joint.find('parent').get('link')
    expected = ['ur5e_'+name for name in ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']]
    if list(reversed(actuated)) != expected:
        raise RuntimeError('SRDF manipulator must contain the six BRL ur5e_ joints in order')
    return ET.tostring(semantic, encoding='unicode')


def from_master(master, namespace, fallback):
    caller = '/prepare_field_moveit_semantics'
    code, message, urdf = master.getParam(caller, namespace+'/robot_description')
    if code != 1:
        raise RuntimeError('Live robot_description unavailable; start BRL bringup first: '+message)
    code, message, srdf = master.getParam(caller, namespace+'/robot_description_semantic')
    used_fallback = code != 1
    if used_fallback:
        srdf = Path(fallback).read_text()
    return normalize(urdf, srdf), used_fallback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-namespace', default='/brl_robot')
    parser.add_argument('--fallback-srdf', type=Path)
    args = parser.parse_args()
    namespace = args.robot_namespace.rstrip('/')
    master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
    fallback = args.fallback_srdf
    if fallback is None:
        import rospkg
        fallback = Path(rospkg.RosPack().get_path('summit_ur5_hardware_setup'))/'field/ur5e_moveit.srdf'
    semantic, used_fallback = from_master(master, namespace, fallback)
    if used_fallback:
        print('BRL semantic parameter missing; using validated fallback '+str(fallback), file=sys.stderr)
    sys.stdout.write(semantic)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Write mobile_manipulator.urdf with the lab UR5e arm ranges for the BRL Pick sim.

The sim URDF limits every arm joint to +-180 deg; the lab planning model (and the real UR5e)
allows +-360 deg (elbow +-160 deg, hardware_setup/brl_lab/make_brl_lab_summit.py). With +-180
the pre-pick IK for the lab tomato can land on wrist_1 = 180 deg and the 2 cm visual-servoing
steps then hit the bound. MuJoCo already models +-360 (elbow +-180).
Used from brl_pick_sim.launch as a param command (runs before any node starts); prints the
output path, which becomes the param value.
usage: make_brl_pick_sim_urdf.py OUT [--elbow-limit-deg 160]
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

ARM = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('out')
    parser.add_argument('--elbow-limit-deg', type=float, default=160.0)
    args, _ = parser.parse_known_args()  # roslaunch may append __name/__log
    source = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'mobile_manipulator.urdf')
    tree = ET.parse(source)
    changed = 0
    for joint in tree.getroot().findall('joint'):
        name = joint.get('name', '')
        if name in ['ur5e_%s_joint' % a for a in ARM]:
            bound = math.radians(args.elbow_limit_deg) if name == 'ur5e_elbow_joint' else 2 * math.pi
            limit = joint.find('limit')
            limit.set('lower', repr(-bound))
            limit.set('upper', repr(bound))
            changed += 1
    if changed != 6:
        raise SystemExit('expected 6 arm joints in %s, found %d' % (source, changed))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    tree.write(args.out, encoding='utf-8', xml_declaration=True)
    print(args.out)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""BRL_lab: remove BRL's basket collision objects from the planning scene.

The BRL motion planner (unchanged) adds a basket behind the arm in setup_scene();
the lab has no basket. This waits until those objects appear and removes them.
Planning scene only: nothing is sent to the robot.
"""
import sys

import moveit_commander
import rospy

BASKET = ['basket_bottom', 'basket_right_wall', 'basket_left_wall', 'basket_back_wall',
          'sub_basket_left', 'sub_basket_right']


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('brl_lab_remove_basket')
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    timeout = rospy.get_param('~timeout', 120.0)
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown():
        present = [name for name in BASKET if name in scene.get_known_object_names()]
        if present:
            rospy.sleep(1.0)  # let setup_scene finish adding every part
            present = [name for name in BASKET if name in scene.get_known_object_names()]
            for name in present:
                scene.remove_world_object(name)
            rospy.loginfo('[BRL_lab] Removed BRL basket objects: %s' % ', '.join(present))
            return
        if rospy.Time.now() > deadline:
            rospy.logwarn('[BRL_lab] No BRL basket objects appeared within %.0f s' % timeout)
            return
        rospy.sleep(0.5)


if __name__ == '__main__':
    main()

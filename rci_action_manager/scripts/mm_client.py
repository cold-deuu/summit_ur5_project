#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import geometry_msgs.msg
import actionlib
from actionlib_msgs.msg import GoalStatus
import rci_action_manager.msg
import cmd, os
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf
from tf.transformations import quaternion_matrix
import math
from std_msgs.msg import Float64
from copy import deepcopy 
from gazebo_msgs.msg import LinkState
#import eigenpy
#eigenpy.switchToNumpyArray()


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "
    def __init__(self):
            cmd.Cmd.__init__(self)
            rospy.init_node('cdpr_actions_client')
            
            self.se3_ctrl_client = actionlib.SimpleActionClient('rci_action_manager/se3Control', rci_action_manager.msg.SE3Action)
            self.se3_ctrl_client.wait_for_server()
            self.joint_ctrl_client = actionlib.SimpleActionClient('rci_action_manager/jointControl', rci_action_manager.msg.JointPostureAction)
            self.joint_ctrl_client.wait_for_server()

            # Joint Trajectory Msgs 받는 코드 작성
            # rospy.Subscriber = 
            # self.traj_cmd = 


    def _send_and_wait(self, client, goal):
        try:
            client.send_goal(goal)
            client.wait_for_result()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            client.cancel_goal()
            raise
        result = client.get_result()
        if client.get_state() == GoalStatus.SUCCEEDED and result is not None and result.is_completed:
            print("action succeed")
        else:
            print("action failed (state=%s, result=%s, reason=%s)" %
                  (client.get_state(), result, client.get_goal_status_text()))

    def do_home(self, arg):
        print("JointPosture Control")
        goal = rci_action_manager.msg.JointPostureGoal()
        goal.duration = 5.0
        goal.target_joints = JointState()
        # goal.target_joints.position = np.array([np.pi, -1.57/2, 1.57,0.0, 0.0, 0.0])
        # goal.target_joints.position = np.array([1.57, -2.0, 1.57,-1.57, -1.57, 0.0])

        # Field home pan: +180 deg, close to the measured +185 deg branch.
        goal.target_joints.position = np.array([np.pi, -1.57, -np.pi/2, -1.57, np.pi/2, 0.0])
        # goal.target_joints.position = np.array([0, 0, 0, 0, 0, 0])

        self._send_and_wait(self.joint_ctrl_client, goal)

    def do_wholebody(self, arg):
        print ("Wholebody Control")
        goal = rci_action_manager.msg.SE3Goal()
        goal.duration = 3
        goal.target_pose = Pose()
        
        goal.target_pose.position.x = 0.2

        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = -0.2

        goal.target_pose.orientation.x = 0.0
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0.0
        goal.target_pose.orientation.w = 1.0


        goal.relative = True
        goal.wholebody = True

        self._send_and_wait(self.se3_ctrl_client, goal)
            

    def do_wholebody2(self, arg):
        print ("Wholebody Control")
        goal = rci_action_manager.msg.SE3Goal()
        goal.duration = 3
        goal.target_pose = Pose()
        
        goal.target_pose.position.x = +0.3
        goal.target_pose.position.y = 0.3
        goal.target_pose.position.z = 0.0

        goal.target_pose.orientation.x = 0.7071068
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0.0
        goal.target_pose.orientation.w = 0.7071068


        goal.relative = True
        goal.wholebody = True

        self._send_and_wait(self.se3_ctrl_client, goal)


    def do_sca_se3(self, arg):
        print ("Wholebody Control")
        goal = rci_action_manager.msg.SE3Goal()
        goal.duration = 5
        goal.target_pose = Pose()
        
        goal.target_pose.position.x = -0.4

        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = -0.6

        goal.target_pose.orientation.x = 0.0
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0.0
        goal.target_pose.orientation.w = 1.0


        goal.relative = True
        goal.wholebody = True

        self._send_and_wait(self.se3_ctrl_client, goal)

    def do_sca_joint(self, arg):
        print("JointPosture Control")
        goal = rci_action_manager.msg.JointPostureGoal()
        goal.duration = 13.0
        goal.target_joints = JointState()
        goal.target_joints.position = np.array([np.pi, -1.57/2, 1.57,0.0, 0.0, 0.0])
        # goal.target_joints.position = np.array([0, -1.57, 1.57,-1.57, -1.57, 0.0])

        self._send_and_wait(self.joint_ctrl_client, goal)

    def do_quit(self, arg):
          'do quit'
          return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()

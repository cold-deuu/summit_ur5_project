from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import geometry_msgs.msg
import actionlib
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


    def do_home(self, arg):
        print("JointPosture Control")
        goal = rci_action_manager.msg.JointPostureGoal
        goal.duration = 3.0
        goal.target_joints = JointState()
        # goal.target_joints.position = np.array([0.0, -1.57/2, 1.57,0.0, 0.0, 0.0])
        goal.target_joints.position = np.array([0.0, -3.14/2, 1.57, -1.57 , -1.57, 0.0])



        self.joint_ctrl_client.send_goal(goal)
        self.joint_ctrl_client.wait_for_result()
        if (self.joint_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_wholebody(self, arg):
        print ("Wholebody Control")
        goal = rci_action_manager.msg.SE3Goal
        goal.duration = 8
        goal.target_pose = Pose()
        
        goal.target_pose.position.x = -1.0
        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = -0.8

        goal.target_pose.orientation.x = 0
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0.0
        goal.target_pose.orientation.w = 1.0


        goal.relative = True
        goal.wholebody = True

        self.se3_ctrl_client.send_goal(goal)
        self.se3_ctrl_client.wait_for_result()
        if (self.se3_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
            

    def do_wholebody2(self, arg):
        print ("Wholebody Control")
        goal = rci_action_manager.msg.SE3Goal
        goal.duration = 10
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

        self.se3_ctrl_client.send_goal(goal)
        self.se3_ctrl_client.wait_for_result()
        if (self.se3_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
            


    def do_quit(self, arg):
          'do quit'
          return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()

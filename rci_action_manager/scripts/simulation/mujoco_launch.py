#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
from mujoco import viewer

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class mujocoSim:
    def __init__(self):
        # XML 로딩
        self.model = mujoco.MjModel.from_xml_path("/home/chan/catkin_ws/src/robot_description/universal_robots_ur5e/summit_ur5e_sim.xml")
        self.data = mujoco.MjData(self.model)
        self.joint_indices = {name: i for i, name in enumerate(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                                                        for i in range(self.model.njnt))}

        self.num_joints = self.model.nu
        self.ctrl = np.zeros((self.num_joints))

        rospy.init_node("mujoco_summit_ur") 
        rospy.Subscriber("/mujoco_ctrl", Float64MultiArray, self.mujoco_command_callback)
        self.joint_states_pub = rospy.Publisher("/mujoco/joint_states", JointState,queue_size=10)

        self.rate = rospy.Rate(100)

        self.initSimulation = True
        self.controlFlag = False

    def mujoco_command_callback(self, msg):
        self.controlFlag = True
        for i in range(self.num_joints):
            self.ctrl[i] = msg.data[i]
    
    def joint_states_publish(self, qpos, qvel):
        joint_states = JointState()
        joint_states.position = qpos.tolist()
        joint_states.velocity = qvel.tolist()
        self.joint_states_pub.publish(joint_states)
        

    def main(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while not rospy.is_shutdown() and viewer.is_running():
                self.joint_states_publish(self.data.qpos, self.data.qvel)

                if self.initSimulation:
                    self.data.qpos = np.array([0.0, 0.0, 0.0, 0.0, -1.57/2, 1.57,0.0, 0.0, 0.0])
                    mujoco.mj_forward(self.model, self.data)
                    self.initSimulation = False
                if self.controlFlag:
                    self.data.qpos[:] = self.ctrl  # 컨트롤 값 적용
                    self.data.ctrl[:] = self.ctrl  # 컨트롤 값 적용
                    # mujoco.mj_kinematics(self.model, self.data)
                    mujoco.mj_step(self.model, self.data)
                viewer.sync()
                # self.rate.sleep()


if __name__ == "__main__":
    mjSim = mujocoSim()
    mjSim.main()
#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
from mujoco import viewer

import rospy
import rospkg

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
class mujocoSim:
    def __init__(self):
        rospy.init_node("mujoco_summit_ur") 
        rospy.Subscriber("/mujoco_ctrl", Float64MultiArray, self.mujoco_command_callback)
        self.is_wholebody = rospy.get_param("/whole_body", False)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("summit_xl_description") + "/.."

        if not self.is_wholebody:
            urdf_path = "/universal_robots_ur5e/scene.xml"
        else:
            urdf_path = "/universal_robots_ur5e/summit_ur5e_sim.xml"

        self.model = mujoco.MjModel.from_xml_path(rospy.get_param("~model_file", pkg_path + urdf_path))
        self.data = mujoco.MjData(self.model)
        # self.joint_indices = {name: i for i, name in enumerate(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
        #                                                 for i in range(self.model.njnt))}

        # self.num_joints = self.model.nu
        # self.ctrl = np.zeros((self.num_joints))

        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        
        if key_id != -1: # "home" 키프레임이 존재한다면
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
            print(f"Keyframe 'home' loaded successfully. (ID: {key_id})")
        else:
            print("Warning: Keyframe 'home' not found!")
        # ---------------------------------------------------------

        self.joint_indices = {name: i for i, name in enumerate(mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
                                                        for i in range(self.model.njnt))}

        self.num_joints = self.model.nu

        if key_id != -1:
            self.data.ctrl = self.model.key_ctrl[key_id].copy()
        else:
            self.data.ctrl = np.zeros((self.num_joints))



        self.joint_states_pub = rospy.Publisher("/mujoco/joint_states", JointState,queue_size=10)
        self.ft_sensor_pub = rospy.Publisher("/mujoco/ft_sensor", Float64MultiArray,queue_size=10)


        self.rate = rospy.Rate(100)

        self.initSimulation = True
        self.controlFlag = False

        self.ft_sensor = np.zeros((6)) # F/T Sensor 

        # self.sid_f = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, b"ee_force")
        # self.sid_t = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, b"ee_torque")

        # self.adr_f = self.model.sensor_adr[self.sid_f]; self.dim_f = self.model.sensor_dim[self.sid_f]   # dim_f=3
        # self.adr_t = self.model.sensor_adr[self.sid_t]; self.dim_t = self.model.sensor_dim[self.sid_t]   # dim_t=3


    def mujoco_command_callback(self, msg):
        self.controlFlag = True

        for i in range(self.num_joints):
            self.data.ctrl[i] = msg.data[i]
        # self.data.ctrl[:3] += self.ctrl[:3] * 0.01

    def joint_states_publish(self, qpos, qvel):
        joint_states = JointState()
        joint_states.position = qpos.tolist()
        joint_states.velocity = qvel.tolist()
        self.joint_states_pub.publish(joint_states)
    
    def ft_sensor_publish(self):
        # 0) 센서 읽기: 이미 ee_site(EEF) 로컬 프레임 기준의 값임!
        F_eef = self.data.sensor('ee_force').data.copy()   # [Fx, Fy, Fz] @ EEF
        T_eef = self.data.sensor('ee_torque').data.copy()  # [Tx, Ty, Tz] @ EEF

        gravity = np.array([0, 0, -9.81])

        # 1) 바로 메시지에 담아서 퍼블리시
        self.ft_sensor[:3] = F_eef
        self.ft_sensor[3:] = T_eef
        
        ft_msg = Float64MultiArray()
        ft_msg.data = self.ft_sensor.tolist()
        self.ft_sensor_pub.publish(ft_msg)
        # 소수점 4자리까지 깔끔하게 포맷팅해서 출력
        str_F = ", ".join([f"{x:.4f}" for x in self.ft_sensor[:3]])
        str_T = ", ".join([f"{x:.4f}" for x in self.ft_sensor[3:]])
        
        # print(f"F_eef : [{str_F}]")
        # print(f"T_eef : [{str_T}]")
    
    def compute_ext_force(self):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'wrist_3_link')
        
        w_body = self.data.cfrc_ext[bid].copy()
        
        T_global = w_body[:3]
        F_global = w_body[3:]

        R_wb = self.data.xmat[bid].reshape(3, 3) 
        R_bw = R_wb.T                            
        
        F_local = R_bw @ F_global
        T_local = R_bw @ T_global

        ft_sensor = np.concatenate((F_local, T_local), axis=0)
        ft_msg = Float64MultiArray()
        ft_msg.data = ft_sensor.tolist()
        self.ft_sensor_pub.publish(ft_msg)
        
        str_F = ", ".join([f"{x:.4f}" for x in F_local])
        str_T = ", ".join([f"{x:.4f}" for x in T_local])
        print(f"Pure Ext Force (Local) : [{str_F}]")

    # def main(self):
    #     with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
    #         while not rospy.is_shutdown() and viewer.is_running():
    #             self.joint_states_publish(self.data.qpos, self.data.qvel)

    #             self.compute_ext_force()
    #             if self.initSimulation:
    #                 self.data.ctrl = np.zeros((self.num_joints))
    #                 mujoco.mj_step(self.model, self.data)

    #                 self.initSimulation = False
                
    #             if self.controlFlag:
    #                 self.data.qpos[3:] = self.ctrl[3:]
    #                 mujoco.mj_forward(self.model, self.data)

    #             viewer.sync()


    # def main(self):
    #     with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
    #         while not rospy.is_shutdown() and viewer.is_running():
    #             self.joint_states_publish(self.data.qpos, self.data.qvel)
    #             self.compute_ext_force()
                
    #             if self.initSimulation:
    #                 self.data.ctrl = np.zeros((self.num_joints))
    #                 # mujoco.mj_step(self.model, self.data)
    #                 self.initSimulation = False
                
    #             # --- 수정된 부분 시작 ---
    #             if self.controlFlag:
    #                 for i in range(self.num_joints):
    #                     self.data.ctrl[i] = self.ctrl[i]
                    
    #             #     mujoco.mj_step(self.model, self.data)
    #             # else:
    #             #     mujoco.mj_step(self.model, self.data)

    #             viewer.sync()


    # def main(self):
    #     # 로봇이 쳐지지 않도록 MuJoCo 월드의 중력을 0으로 설정합니다.
    #     # self.model.opt.gravity[:] = [0, 0, 0]
        
    #     with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
    #         while not rospy.is_shutdown() and viewer.is_running():
    #             self.joint_states_publish(self.data.qpos, self.data.qvel)
    #             self.ft_sensor_publish()

    #             mujoco.mj_step(self.model, self.data)

    #             viewer.sync()
    #             self.rate.sleep()

    def main(self):
        sim_dt = self.model.opt.timestep 
        self.model.opt.gravity[:] = [0, 0, 0]

        ros_dt = 1.0 / 100.0 
        
        steps_per_ros_loop = int(ros_dt / sim_dt)
        print(f"Physics steps per ROS loop: {steps_per_ros_loop}")
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while not rospy.is_shutdown() and viewer.is_running():
                
                self.joint_states_publish(self.data.qpos, self.data.qvel)
                self.ft_sensor_publish()
                # self.compute_ext_force()

                for _ in range(steps_per_ros_loop):
                    mujoco.mj_step(self.model, self.data)

                viewer.sync()
                self.rate.sleep()


if __name__ == "__main__":
    mjSim = mujocoSim()
    mjSim.main()
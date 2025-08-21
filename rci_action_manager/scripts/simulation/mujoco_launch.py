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
        self.ft_sensor_pub = rospy.Publisher("/mujoco/ft_sensor", Float64MultiArray,queue_size=10)

        self.rate = rospy.Rate(100)

        self.initSimulation = True
        self.controlFlag = False

        self.ft_sensor = np.zeros((6)) # F/T Sensor 

        self.sid_f = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, b"ee_force")
        self.sid_t = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, b"ee_torque")

        self.adr_f = self.model.sensor_adr[self.sid_f]; self.dim_f = self.model.sensor_dim[self.sid_f]   # dim_f=3
        self.adr_t = self.model.sensor_adr[self.sid_t]; self.dim_t = self.model.sensor_dim[self.sid_t]   # dim_t=3


    def mujoco_command_callback(self, msg):
        self.controlFlag = True
        for i in range(self.num_joints):
            self.ctrl[i] = msg.data[i]
    
    def joint_states_publish(self, qpos, qvel):
        joint_states = JointState()
        joint_states.position = qpos.tolist()
        joint_states.velocity = qvel.tolist()
        self.joint_states_pub.publish(joint_states)
    
    # def ft_sensor_publish(self):
    #     # F = self.data.sensordata[self.adr_f:self.adr_f+self.dim_f]    # [Fx, Fy, Fz] in ee_site frame
    #     # T = self.data.sensordata[self.adr_t:self.adr_t+self.dim_t]    # [Tx, Ty, Tz] in ee_site frame
    #     F = self.data.sensor('ee_force').data.copy()
    #     T = self.data.sensor('ee_torque').data.copy()

    #     print("Mujoco" + "-" * 20)
    #     print("Force:", F)
    #     print("Torque:", T)

    #     # self.ft_sensor = np.concatenate((F, T))  # Concatenate force and torque        
    #     self.ft_sensor[:3] = F
    #     self.ft_sensor[3:] = T
    #     ft_msg = Float64MultiArray()
    #     ft_msg.data = self.ft_sensor.tolist()
    #     self.ft_sensor_pub.publish(ft_msg)


    #     # 오른손 좌표계
    #     # sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    #     # R = self.data.site_xmat[sid].reshape(3,3)

    #     # print("det(R) =", np.linalg.det(R))        # ➜ 반드시 +1 근처
    #     # x, y, z = R[:,0], R[:,1], R[:,2]
    #     # print("x×y - z =", np.cross(x, y) - z)  


    def ft_sensor_publish(self):
        # 0) 센서 읽기: site 프레임 기준
        F_site = self.data.sensor('ee_force').data.copy()   # [Fx, Fy, Fz] @ site
        T_site = self.data.sensor('ee_torque').data.copy()  # [Tx, Ty, Tz] @ site

        # 1) 회전행렬 준비
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'ee_site')
        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base')  # 원하는 기준 바디명

        R_ws = self.data.site_xmat[site_id].reshape(3, 3)  # site -> world
        R_wb = self.data.xmat[base_id].reshape(3, 3)       # base -> world
        R_bw = R_wb.T                                      # world -> base

        # 2) site -> base 변환 (한 번에 R_sb = R_bw @ R_ws)
        R_sb = R_bw @ R_ws
        F_base = R_sb @ F_site
        T_base = R_sb @ T_site

        # 4) 메시지 퍼블리시 (Float64MultiArray 유지 시)
        self.ft_sensor[:3] = F_base
        self.ft_sensor[3:] = T_base
        ft_msg = Float64MultiArray()
        ft_msg.data = self.ft_sensor.tolist()
        self.ft_sensor_pub.publish(ft_msg)


        # print(f"Rotation : {R_sb}")

        # print(f"Force: {F_base}, Torque: {T_base}")
        # print(f"Force: {F_base}, Torque: {T_base}")


    def compute_ext_force(self):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'writs_3_link')  # EE가 있는 바디명
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'ee_site')
        # 바디 프레임의 외력 합력 [Fx,Fy,Fz,Tx,Ty,Tz]
        w_body = self.data.cfrc_ext[bid].copy()
        # print(f"W body :{w_body}")

        F_b = w_body[3:]; T_b = w_body[:3]
        # print(f"Force : {F_b}, Torque : {T_b}")
        ft_sensor = np.concatenate((F_b, T_b), axis=0)
        ft_msg = Float64MultiArray()
        ft_msg.data=  ft_sensor.tolist()
        self.ft_sensor_pub.publish(ft_msg)
        

    def main(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while not rospy.is_shutdown() and viewer.is_running():
                self.joint_states_publish(self.data.qpos, self.data.qvel)
                # EE
                self.compute_ext_force()
                if self.initSimulation:
                    self.data.qpos = np.array([0.0, 0.0, 0.0, 0.0, -1.57/2, 1.57,0.0, 0.0, 0.0])
                    mujoco.mj_forward(self.model, self.data)
                    self.initSimulation = False
                
                if self.controlFlag:
                    # self.data.qpos[:] = self.ctrl  # 컨트롤 값 적용
                    self.data.ctrl[:] = self.ctrl  # 컨트롤 값 적용
                    mujoco.mj_step(self.model, self.data)
                # else:
                #     mujoco.mj_forward(self.model, self.data)


                
                viewer.sync()
                # self.rate.sleep()


if __name__ == "__main__":
    mjSim = mujocoSim()
    mjSim.main()
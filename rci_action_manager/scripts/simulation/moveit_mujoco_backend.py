#!/usr/bin/env python3
"""Named MuJoCo mapping for arm-only, mobile MoveIt, and mobile ROS actions."""
import os
os.environ.setdefault("MUJOCO_GL","disable")
import importlib.util
import json
from pathlib import Path
import time
import mujoco
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
import tf

NAMES=["ur5e_shoulder_pan_joint","ur5e_shoulder_lift_joint","ur5e_elbow_joint",
       "ur5e_wrist_1_joint","ur5e_wrist_2_joint","ur5e_wrist_3_joint"]
MJ_NAMES=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
BASE_NAMES=["v_x_joint","v_y_joint","v_t_joint"]
MJ_BASE=["base_x_slide_joint","base_y_slide_joint","base_z_hinge_joint"]

def main():
    spec=importlib.util.spec_from_file_location("legacy_mujoco",Path(__file__).with_name("mujoco_launch.py"))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    class MappedSimulator(module.mujocoSim):
        def __init__(self):
            self.mapping_ready=False
            super().__init__()
            self.mobile=rospy.get_param("~mobile_base",False)
            self.base_mode=rospy.get_param("~base_mode","fixed")
            if self.base_mode not in ("fixed","commanded"):raise ValueError("base_mode")
            self.arm_ids=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in MJ_NAMES]
            self.base_ids=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in MJ_BASE] if self.mobile else []
            if min(self.arm_ids+self.base_ids)<0:raise ValueError("Missing MuJoCo joint name")
            def actuator(joint):
                matches=np.where(self.model.actuator_trnid[:,0]==joint)[0]
                if len(matches)!=1:raise ValueError("Expected one actuator for joint")
                return int(matches[0])
            self.arm_actuators=[actuator(j) for j in self.arm_ids]
            self.base_actuators=[actuator(j) for j in self.base_ids]
            self.arm_q=[int(self.model.jnt_qposadr[j]) for j in self.arm_ids]
            self.arm_v=[int(self.model.jnt_dofadr[j]) for j in self.arm_ids]
            self.base_q=[int(self.model.jnt_qposadr[j]) for j in self.base_ids]
            self.base_v=[int(self.model.jnt_dofadr[j]) for j in self.base_ids]
            self.base_reference=self.data.qpos[self.base_q].copy()
            self.base_command=np.zeros(3);self.last_command=0
            self.mapping_ready=True
        def mujoco_command_callback(self,msg):
            if not self.mapping_ready:return
            values=np.asarray(msg.data,dtype=float)
            if not np.isfinite(values).all():return
            expected=9 if self.mobile and self.base_mode=="commanded" else 6
            if len(values)!=expected:
                rospy.logwarn_throttle(1,"Expected %d commands, got %d",expected,len(values));return
            if expected==9:self.base_command[:]=values[:3]
            self.data.ctrl[self.arm_actuators]=values[-6:]
            self.last_command=time.monotonic()
        def ft_sensor_publish(self):
            if not self.mobile:
                return super().ft_sensor_publish()
            # The uploaded tool adds inertial loads to the legacy uncalibrated
            # wrist sensor. This test backend exposes the known external wrench
            # (load-compensated sensor), rather than feeding tool inertia back
            # into admittance. Contact/grasp sensing is not modeled here.
            from std_msgs.msg import Float64MultiArray
            bid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,"wrist_3_link")
            rotation=self.data.site("ee_site").xmat.reshape(3,3)
            self.ft_sensor[:3]=-rotation.T @ self.data.xfrc_applied[bid,:3]
            self.ft_sensor[3:]=-rotation.T @ self.data.xfrc_applied[bid,3:]
            self.ft_sensor_pub.publish(Float64MultiArray(data=self.ft_sensor.tolist()))
        def update_base(self):
            if not self.mobile:return
            if self.base_mode=="fixed":
                command=np.clip(5*(self.base_reference-self.data.qpos[self.base_q]),-.2,.2)
            else:
                command=self.base_command if time.monotonic()-self.last_command<.25 else np.zeros(3)
            self.data.ctrl[self.base_actuators]=command
    sim=MappedSimulator()
    sim.model.opt.gravity[:]=0
    sim.data.qpos[sim.arm_q]=[0,-1.57,1.57,-1.57,-1.57,0]
    sim.data.ctrl[sim.arm_actuators]=sim.data.qpos[sim.arm_q]
    mujoco.mj_forward(sim.model,sim.data)
    headless=rospy.get_param("~headless",True);mock=rospy.get_param("~mock_real",False)
    if mock:
        def command(msg):
            if len(msg.points)!=1 or set(msg.joint_names)!=set(NAMES):return
            positions=msg.points[0].positions
            if len(positions)!=6 or not np.isfinite(positions).all():return
            sim.data.ctrl[sim.arm_actuators]=[positions[msg.joint_names.index(n)] for n in NAMES]
            sim.last_command=time.monotonic()
        rospy.Subscriber("/mock_hw/command",JointTrajectory,command,queue_size=1)
    state_pub=rospy.Publisher("/mock_hw/joint_states" if mock else "/mujoco/joint_states",JointState,queue_size=1)
    planning_pub=rospy.Publisher("/joint_states",JointState,queue_size=1)
    wrench_pub=rospy.Publisher("/mock_hw/wrench",WrenchStamped,queue_size=1)
    base_pub=rospy.Publisher("/mujoco/base_state",Odometry,queue_size=1)
    broadcaster=tf.TransformBroadcaster()
    info_pub=rospy.Publisher("/mujoco/mapping_info",String,queue_size=1,latch=True)
    info_pub.publish(String(data=json.dumps(dict(
        nq=sim.model.nq,nv=sim.model.nv,nu=sim.model.nu,mobile_base=sim.mobile,
        base_mode=sim.base_mode,arm_qpos_addresses=sim.arm_q,arm_actuator_ids=sim.arm_actuators,
        base_qpos_addresses=sim.base_q,base_actuator_ids=sim.base_actuators,
        arm_joint_mapping=dict(zip(NAMES,MJ_NAMES))))))
    passive=rospy.get_param("~passive_joints",[])
    external=np.zeros(6);external_stamp=[0.]
    def push(msg):
        external[:]=[msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,
                     msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z]
        external_stamp[0]=time.monotonic()
    rospy.Subscriber("/mujoco/apply_wrench",WrenchStamped,push,queue_size=1)
    wrist=mujoco.mj_name2id(sim.model,mujoco.mjtObj.mjOBJ_BODY,"wrist_3_link")
    viewer=None
    if not headless:
        from mujoco import viewer as mjviewer
        viewer=mjviewer.launch_passive(sim.model,sim.data)
    rate=rospy.Rate(100)
    while not rospy.is_shutdown() and (viewer is None or viewer.is_running()):
        sim.data.xfrc_applied[:]=0
        if time.monotonic()-external_stamp[0]<.15:sim.data.xfrc_applied[wrist,:]=external
        sim.update_base()
        now=rospy.Time.now()
        arm_q=sim.data.qpos[sim.arm_q].tolist();arm_v=sim.data.qvel[sim.arm_v].tolist()
        planning=JointState()
        planning.header.stamp=now;planning.name=NAMES+passive
        planning.position=arm_q+[0.]*len(passive);planning.velocity=arm_v+[0.]*len(passive)
        planning_pub.publish(planning)
        state=JointState()
        state.header.stamp=now
        state.name=(BASE_NAMES if sim.mobile else [])+planning.name
        state.position=(sim.data.qpos[sim.base_q].tolist() if sim.mobile else [])+planning.position
        state.velocity=(sim.data.qvel[sim.base_v].tolist() if sim.mobile else [])+planning.velocity
        state_pub.publish(state)
        if sim.mobile:
            x,y,yaw=sim.data.qpos[sim.base_q];vx,vy,w=sim.data.qvel[sim.base_v]
            q=(0,0,float(np.sin(yaw/2)),float(np.cos(yaw/2)))
            odom=Odometry();odom.header.stamp=now;odom.header.frame_id="world"
            odom.child_frame_id="summit_xl_base_footprint"
            odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
            odom.pose.pose.orientation.z=q[2];odom.pose.pose.orientation.w=q[3]
            odom.twist.twist.linear.x=np.cos(yaw)*vx+np.sin(yaw)*vy
            odom.twist.twist.linear.y=-np.sin(yaw)*vx+np.cos(yaw)*vy
            odom.twist.twist.angular.z=w;base_pub.publish(odom)
            broadcaster.sendTransform((x,y,0),q,now,"summit_xl_base_footprint","world")
        sim.ft_sensor_publish()
        if mock:
            msg=WrenchStamped();msg.header.stamp=now;msg.header.frame_id="ee_site"
            f=sim.ft_sensor
            msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z=f[:3]
            msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z=f[3:]
            wrench_pub.publish(msg)
        for _ in range(round(.01/sim.model.opt.timestep)):
            mujoco.mj_step(sim.model,sim.data)
            if sim.mobile and sim.base_mode=="fixed":
                sim.data.qpos[sim.base_q]=sim.base_reference
                sim.data.qvel[sim.base_v]=0
        if sim.mobile and sim.base_mode=="fixed":mujoco.mj_forward(sim.model,sim.data)
        if viewer is not None:viewer.sync()
        rate.sleep()
if __name__=="__main__":main()

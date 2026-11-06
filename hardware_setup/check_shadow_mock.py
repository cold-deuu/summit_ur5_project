#!/usr/bin/env python3
"""Synthetic inputs only; must run on a separate master with no hardware driver."""
import json,time
from pathlib import Path
import numpy as np
import rospy,tf
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from rci_action_manager.msg import MoveitOptimalStatus
rospy.init_node('check_shadow_mock')
assert rospy.get_param('/ur_hardware_interface/robot_ip',None) is None,'Use isolated mock master only'
status=[];commands=[];wrenches=[]
rospy.Subscriber('/moveit_optimal_controller/status',MoveitOptimalStatus,status.append)
rospy.Subscriber('/hqp_shadow/unused_command',JointTrajectory,commands.append)
rospy.Subscriber('/hqp_shadow/wrench',WrenchStamped,wrenches.append)
qpub=rospy.Publisher('/joint_states',JointState,queue_size=5)
fpub=rospy.Publisher('/wrench',WrenchStamped,queue_size=5);broadcast=tf.TransformBroadcaster()
start=time.monotonic();rate=rospy.Rate(200)
while time.monotonic()-start<5:
 stamp=rospy.Time.now()
 q=JointState();q.header.stamp=stamp
 q.name=['elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
 q.position=[.00045,-1.57073,-.00192,-1.56530,.00226,-.00490];q.velocity=[0.]*6;qpub.publish(q)
 broadcast.sendTransform((.1,0,0),tf.transformations.quaternion_from_euler(0,0,np.pi/2),stamp,'tool0_controller','wrist_3_link')
 f=WrenchStamped();f.header.stamp=stamp;f.header.frame_id='tool0_controller'
 f.wrench.force.x=1;f.wrench.force.y=-.5;f.wrench.force.z=.3;fpub.publish(f)
 rate.sleep()
assert status and wrenches
s=status[-1];assert s.ft_bias_ready and s.state=='HOLDING' and not s.command_enabled
assert not commands,'Shadow published a command'
assert np.max(np.abs(np.array(s.actual)-[-.00192,-1.57073,.00045,-1.56530,.00226,-.00490]))<1e-9
w=wrenches[-1].wrench
assert np.max(np.abs(np.array([w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z])-[.5,1,.3,0,-.03,.1]))<1e-8
result=dict(passed=True,status_samples=len(status),wrench_samples=len(wrenches),hardware_command_messages=len(commands),state=s.state,ft_bias_ready=s.ft_bias_ready,reason=s.reason,joint_name_order_verified=True,wrench_rotation_and_moment_shift_verified=True)
root=Path(__file__).resolve().parent;(root/'shadow_mock_result.json').write_text(json.dumps(result,indent=2));print(result)

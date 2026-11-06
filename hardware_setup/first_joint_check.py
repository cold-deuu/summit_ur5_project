#!/usr/bin/env python3
"""Preview by default. --execute explicitly sends a 1-degree UR shoulder goal."""
import argparse
import json
import math
import xml.etree.ElementTree as ET
import rospy
from sensor_msgs.msg import JointState

NAMES=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
       'wrist_1_joint','wrist_2_joint','wrist_3_joint']

def ordered_state(msg,now):
    if len(msg.position)!=len(msg.name) or len(msg.velocity)!=len(msg.name):
        raise ValueError('Incomplete joint state')
    if not -.05<=now-msg.header.stamp.to_sec()<=.25:
        raise ValueError('Stale/future joint state')
    q=[]
    for name in NAMES:
        if msg.name.count(name)!=1:raise ValueError('Missing/duplicate joint '+name)
        i=msg.name.index(name)
        if not math.isfinite(msg.position[i]) or not math.isfinite(msg.velocity[i]):
            raise ValueError('Nonfinite joint state')
        if abs(msg.velocity[i])>.02:raise ValueError('Robot is moving')
        q.append(msg.position[i])
    return q

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true',help='Actually move shoulder pan +1 degree; other joints hold')
    args=parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('first_joint_check',anonymous=True)
    state=rospy.wait_for_message('/joint_states',JointState,timeout=5)
    q=ordered_state(state,rospy.Time.now().to_sec());goal=list(q);goal[0]+=math.pi/180
    root=ET.fromstring(rospy.get_param('/robot_description'))
    joints={j.get('name'):j for j in root.findall('joint')}
    for name,value in zip(NAMES,goal):
        limit=joints[name].find('limit')
        if not float(limit.get('lower'))+.01<=value<=float(limit.get('upper'))-.01:
            raise ValueError('Target near/outside position limits: '+name)
    print(json.dumps(dict(mode='EXECUTE' if args.execute else 'PREVIEW_ONLY',
        joint_names=NAMES,current_rad=q,target_rad=goal,delta_deg=[1,0,0,0,0,0],
        trajectory_seconds=5,automatic_return=False),indent=2),flush=True)
    if not args.execute:
        print('Preview only: no command/action client created.');return
    # The user must run --execute at the robot after reviewing the preview.
    import actionlib
    from std_msgs.msg import Bool
    from controller_manager_msgs.srv import ListControllers
    from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal
    from trajectory_msgs.msg import JointTrajectoryPoint
    running=rospy.wait_for_message('/ur_hardware_interface/robot_program_running',Bool,timeout=5)
    if not running.data:raise RuntimeError('External Control program is not running')
    rospy.wait_for_service('/controller_manager/list_controllers',timeout=5)
    controllers=rospy.ServiceProxy('/controller_manager/list_controllers',ListControllers)().controller
    if not any(c.name=='scaled_pos_joint_traj_controller' and c.state=='running' for c in controllers):
        raise RuntimeError('Scaled position controller is not running')
    preview_q=list(q)
    client=actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory',FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(5)):raise RuntimeError('Trajectory action unavailable')
    # Re-read after waiting: never use an old preview as the execution start.
    q=ordered_state(rospy.wait_for_message('/joint_states',JointState,timeout=5),rospy.Time.now().to_sec())
    if max(abs(a-b) for a,b in zip(q,preview_q))>.005:raise RuntimeError('State changed since preview; rerun')
    goal=list(q);goal[0]+=math.pi/180
    for name,value in zip(NAMES,goal):
        limit=joints[name].find('limit')
        if not float(limit.get('lower'))+.01<=value<=float(limit.get('upper'))-.01:
            raise ValueError('Refreshed target near/outside position limits: '+name)
    request=FollowJointTrajectoryGoal();request.trajectory.joint_names=NAMES
    request.trajectory.header.stamp=rospy.Time.now()+rospy.Duration(.2)
    for seconds,position in [(0,q),(5,goal)]:
        point=JointTrajectoryPoint();point.positions=position;point.velocities=[0.]*6;point.accelerations=[0.]*6
        point.time_from_start=rospy.Duration(seconds);request.trajectory.points.append(point)
    try:
        client.send_goal(request)
        if not client.wait_for_result(rospy.Duration(130)):raise RuntimeError('Timeout (speed scaling may have paused motion)')
        result=client.get_result()
        if client.get_state()!=3 or result.error_code!=0:raise RuntimeError(str(result))
        print('SUCCEEDED: shoulder moved +1 degree; no automatic return.',flush=True)
    finally:
        if client.get_state() in (0,1,6,7):client.cancel_goal()

if __name__=='__main__':main()

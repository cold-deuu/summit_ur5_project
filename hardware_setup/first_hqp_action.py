#!/usr/bin/env python3
"""Existing real.cpp JointPosture action, +1 degree only; preview by default."""
import argparse,json,math
import rospy,actionlib
from sensor_msgs.msg import JointState
from rci_action_manager.msg import JointPostureAction,JointPostureGoal,MoveitOptimalStatus
from first_joint_check import ordered_state,NAMES

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true')
    a=p.parse_args(rospy.myargv()[1:]);rospy.init_node('first_hqp_action',anonymous=True)
    s=rospy.wait_for_message('/real_action_controller/status',MoveitOptimalStatus,timeout=5)
    if s.reason or not s.ft_bias_ready:raise RuntimeError('Controller not ready: '+s.reason)
    q=ordered_state(rospy.wait_for_message('/joint_states',JointState,timeout=5),rospy.Time.now().to_sec())
    target=list(q);target[0]+=math.pi/180
    print(json.dumps(dict(mode='EXECUTE' if a.execute else 'PREVIEW_ONLY',
        action='/rci_action_manager/jointControl',current_rad=q,target_rad=target,
        delta_deg=[1,0,0,0,0,0],duration_control_seconds=10,command_enabled=s.command_enabled),indent=2),flush=True)
    if not a.execute:return
    if not s.command_enabled:raise RuntimeError('command_enabled is false; no goal sent')
    client=actionlib.SimpleActionClient('/rci_action_manager/jointControl',JointPostureAction)
    if not client.wait_for_server(rospy.Duration(5)):raise RuntimeError('Action unavailable')
    goal=JointPostureGoal();goal.duration=10;goal.target_joints.name=NAMES;goal.target_joints.position=target
    try:
        client.send_goal(goal)
        if not client.wait_for_result(rospy.Duration(360)):raise RuntimeError('Action timeout')
        result=client.get_result()
        if client.get_state()!=3 or not result.is_completed:raise RuntimeError('Action failed: '+str(result))
        final=ordered_state(rospy.wait_for_message('/joint_states',JointState,timeout=5),rospy.Time.now().to_sec())
        error=max(abs(x-y) for x,y in zip(final,target))
        print(json.dumps(dict(result='SUCCEEDED',final_rad=final,max_joint_error_rad=error),indent=2),flush=True)
    finally:
        if client.get_state() in (0,1,6,7):client.cancel_goal()

if __name__=='__main__':main()

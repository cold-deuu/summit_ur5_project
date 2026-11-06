#!/usr/bin/env python3
"""Native UR5e topic trajectory preview; --publish sends one +1 degree test."""
import argparse,json,math,time
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from rci_action_manager.msg import MoveitOptimalStatus
from first_joint_check import ordered_state,NAMES

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--publish',action='store_true')
    args=parser.parse_args(rospy.myargv()[1:]);rospy.init_node('publish_moveit_joint_test',anonymous=True)
    status=rospy.wait_for_message('/moveit_optimal_controller/status',MoveitOptimalStatus,timeout=5)
    if not status.ft_bias_ready or status.reason not in ['', 'goal tolerance satisfied']:
        raise RuntimeError('Controller not ready: '+status.reason)
    if status.state!='HOLDING':raise RuntimeError('An existing trajectory is active')
    q=ordered_state(rospy.wait_for_message('/joint_states',JointState,timeout=5),rospy.Time.now().to_sec())
    target=list(q);target[0]+=math.pi/180
    print(json.dumps(dict(mode='PUBLISH' if args.publish else 'PREVIEW_ONLY',joint_names=NAMES,
                         current_rad=q,target_rad=target,duration_s=5,topic='/rci_action_manager/moveit_plan'),indent=2),flush=True)
    if not args.publish:return
    if not status.command_enabled:raise RuntimeError('Hardware commands are disabled')
    seen=[];sub=rospy.Subscriber('/moveit_optimal_controller/status',MoveitOptimalStatus,seen.append,queue_size=100)
    publisher=rospy.Publisher('/rci_action_manager/moveit_plan',JointTrajectory,queue_size=1,latch=False)
    deadline=time.monotonic()+5
    while not publisher.get_num_connections():
        if time.monotonic()>deadline:raise RuntimeError('No trajectory subscriber')
        time.sleep(.01)
    fresh=ordered_state(rospy.wait_for_message('/joint_states',JointState,timeout=5),rospy.Time.now().to_sec())
    if max(abs(a-b) for a,b in zip(q,fresh))>.005:raise RuntimeError('State moved since preview; retry at standstill')
    message=JointTrajectory();message.joint_names=NAMES
    for seconds,positions in [(0,fresh),(5,target)]:
        point=JointTrajectoryPoint();point.positions=positions;point.velocities=[0.]*6;point.accelerations=[0.]*6
        point.time_from_start=rospy.Duration(seconds);message.points.append(point)
    before=max(status.trajectory_id,status.result_trajectory_id);publisher.publish(message)
    deadline=time.monotonic()+180
    while time.monotonic()<deadline:
        if seen:
            s=seen[-1]
            if s.result_trajectory_id>before and s.last_result in ['REJECTED','ABORTED']:
                raise RuntimeError(s.last_result+': '+s.reason)
            if s.result_trajectory_id>before and s.last_result=='SUCCEEDED' and s.state=='HOLDING':
                print('SUCCEEDED, RMS=%.6f rad, max=%.6f rad'%(s.tracking_rms,s.tracking_max),flush=True);return
        time.sleep(.02)
    # A client timeout is not a controller stop; the task duration is only 5 s.
    raise RuntimeError('Status timeout; inspect controller and stop locally if necessary')

if __name__=='__main__':main()

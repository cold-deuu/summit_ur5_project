#!/usr/bin/env python3
"""Isolated-master test of real.cpp with synthetic driver inputs; no hardware."""
import json,math,threading,time
from pathlib import Path
import actionlib,numpy as np,rospy,tf
from std_msgs.msg import Float64,Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory
from rci_action_manager.msg import JointPostureAction,JointPostureGoal,MoveitOptimalStatus
from first_joint_check import NAMES
rospy.init_node('check_real_actions_mock')
assert rospy.get_param('/ur_hardware_interface/robot_ip',None) is None,'Isolated mock master required'
q=np.array([0,-1.5707,0,-1.5653,.0023,-.0049]);target=q.copy();commands=[];command_times=[];states=[];keep_ft=True;active=True
bad=[];scale=.5;program=True;frozen=False;force_N=0.0

def command(m):
    if m.joint_names!=NAMES or len(m.points)!=1:bad.append('command names/points');return
    expected=rospy.get_param('/real_action_controller/command_horizon')
    if abs(m.points[0].time_from_start.to_sec()-expected)>1e-9:bad.append('incorrect horizon')
    if m.points[0].velocities or m.points[0].accelerations:bad.append('expected position-only command')
    command_times.append(rospy.Time.now().to_sec())
    values=np.array(m.points[0].positions)
    if not np.isfinite(values).all() or np.max(np.abs(values-q))>rospy.get_param('/real_action_controller/max_position_step')+1e-5:bad.append('step/finite')
    target[:]=values;commands.append(m)
rospy.Subscriber('/mock_action/command',JointTrajectory,command)
rospy.Subscriber('/real_action_controller/status',MoveitOptimalStatus,states.append)
qp=rospy.Publisher('/joint_states',JointState,queue_size=1)
wp=rospy.Publisher('/wrench',WrenchStamped,queue_size=1)
sp=rospy.Publisher('/speed_scaling_factor',Float64,queue_size=1)
pp=rospy.Publisher('/ur_hardware_interface/robot_program_running',Bool,queue_size=1,latch=True)
broadcast=tf.TransformBroadcaster()
def backend():
    previous=time.monotonic()
    while active and not rospy.is_shutdown():
        now=time.monotonic();dt=min(.02,now-previous);previous=now
        v=np.zeros(6) if frozen else np.clip((target-q)/.05*scale,-.03,.03);q[:]+=v*dt
        stamp=rospy.Time.now();m=JointState();m.header.stamp=stamp
        order=[2,1,0,3,4,5];m.name=[NAMES[i] for i in order];m.position=q[order].tolist();m.velocity=v[order].tolist();qp.publish(m)
        sp.publish(Float64(scale));pp.publish(Bool(program))
        if keep_ft:
            broadcast.sendTransform((0,0,0),(0,0,0,1),stamp,'tool0_controller','wrist_3_link')
            f=WrenchStamped();f.header.stamp=stamp;f.header.frame_id='tool0_controller';f.wrench.force.x=force_N;wp.publish(f)
        time.sleep(.005)
thread=threading.Thread(target=backend,daemon=True);thread.start()
def wait(predicate,seconds=12):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.01)
    raise RuntimeError('Timeout; last status='+str(states[-1] if states else None))
try:
    wait(lambda:states and states[-1].ft_bias_ready and not states[-1].reason)
    assert np.max(np.abs(np.array(states[-1].actual)-q))<.001
    results=[dict(test='startup_native_names_bias',passed=True)]
    client=actionlib.SimpleActionClient('/rci_action_manager/jointControl',JointPostureAction)
    assert client.wait_for_server(rospy.Duration(5))
    home=JointPostureGoal();home.duration=.1;home.target_joints.name=NAMES
    home.target_joints.position=[0,-1.57,1.57,-1.57,-1.57,0]
    before_home=q.copy();client.send_goal(home)
    assert client.wait_for_result(rospy.Duration(2))
    assert client.get_state()==4 and 'duration infeasible' in client.get_goal_status_text()
    time.sleep(.1);assert np.max(np.abs(q-before_home))<.0001
    results.append(dict(test='infeasible_home_rejected_before_motion',passed=True,reason=client.get_goal_status_text()))
    home.duration=60;client.send_goal(home)
    wait(lambda:client.get_state()==1,2);client.cancel_goal()
    assert client.wait_for_result(rospy.Duration(2)) and client.get_state()==2
    results.append(dict(test='sixty_second_home_accepted_then_cancelled',passed=True))
    if states[-1].command_enabled:
        force_N=6.0;time.sleep(.8);force_N=0.0;time.sleep(1.5)
        assert not states[-1].reason and not bad
        results.append(dict(test='holding_push_release_admittance',passed=True,force_N=6.0,duration_s=.8))
    if not states[-1].command_enabled:
        time.sleep(.5);assert not commands
        results.append(dict(test='command_disabled',passed=True,messages=0))
    else:
        goal=JointPostureGoal();goal.duration=1.;goal.target_joints.name=list(reversed(NAMES))
        desired=q.copy();desired[0]+=math.pi/180;goal.target_joints.position=desired[::-1].tolist()
        before=time.monotonic();client.send_goal(goal)
        assert client.wait_for_result(rospy.Duration(15))
        elapsed=time.monotonic()-before;error=float(np.max(np.abs(q-desired)))
        assert client.get_state()==3 and client.get_result().is_completed,(client.get_state(),error)
        assert elapsed>5.8 and error<.0031 and commands and not bad,(elapsed,error,bad)
        results.append(dict(test='joint_action_scaled_shuffled_names',passed=True,elapsed_s=elapsed,error_max_rad=error,commands=len(commands)))
        malformed=JointPostureGoal();malformed.duration=1;malformed.target_joints.position=[0]
        client.send_goal(malformed);assert client.wait_for_result(rospy.Duration(2));assert client.get_state()==4
        results.append(dict(test='malformed_goal_rejected',passed=True))
        goal.target_joints.position=(q+np.array([.01,0,0,0,0,0]))[::-1].tolist()
        goal.duration=3;client.send_goal(goal);time.sleep(.1);keep_ft=False
        assert client.wait_for_result(rospy.Duration(3));assert client.get_state()==4
        time.sleep(.15);count=len(commands);time.sleep(.35);assert len(commands)==count
        results.append(dict(test='stale_ft_aborts_and_stops_commands',passed=True))
        # Restore valid inputs but force measured state outside the URDF position limits rather than the held
        # command. This must latch a fault, even after the state is restored.
        keep_ft=True
        wait(lambda:states[-1].reason=='',3)
        time.sleep(.1)
        frozen=True;q[0]+=10.0
        wait(lambda:states[-1].reason.startswith('latched fault:'),3)
        time.sleep(.1);count=len(commands);q[0]-=10.0;time.sleep(.35)
        assert len(commands)==count and states[-1].reason.startswith('latched fault:')
        results.append(dict(test='joint_position_fault_latches_without_auto_restart',passed=True,reason=states[-1].reason))

    out=Path(__file__).resolve().parent/('real_actions_mock_result.json' if states[-1].command_enabled else 'real_actions_disabled_mock_result.json')
    if commands:
        period=float(np.median(np.diff(command_times)))
        assert .008<period<.012,period
        results.append(dict(test='position_command_100hz_and_horizon',passed=True,median_period_s=period,
            configured_horizon_s=rospy.get_param('/real_action_controller/command_horizon')))
    out.write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
finally:
    active=False;thread.join(timeout=1)

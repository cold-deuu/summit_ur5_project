#!/usr/bin/env python3
"""Preview a small field HQP action. Only --execute creates an action client."""
import argparse
import json
import math
import rospy
from sensor_msgs.msg import JointState

NAMES = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
         'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']


def current():
    msg = rospy.wait_for_message('/brl_robot/ur5e/joint_states', JointState, timeout=5)
    if not 0 <= (rospy.Time.now() - msg.header.stamp).to_sec() < .25:
        raise RuntimeError('Joint states stale; check PC/robot-host clock synchronization')
    order = [msg.name.index(n) for n in NAMES]
    if len(msg.velocity) != len(msg.name):
        raise RuntimeError('Joint velocity data required')
    q = [msg.position[i] for i in order]
    v = [msg.velocity[i] for i in order]
    if not all(math.isfinite(x) for x in q + v) or max(abs(x) for x in v) > .01:
        raise RuntimeError('Wait for finite, stationary joint state')
    return q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--joint', type=int, choices=range(6), default=0)
    parser.add_argument('--delta-deg', type=float, default=1.)
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.delta_deg) or not 0 < abs(args.delta_deg) <= 5 or not 3 <= args.seconds <= 30:
        parser.error('Use a nonzero delta within +/-5 deg and a duration between 3 and 30 seconds')
    rospy.init_node('field_action_joint_check', anonymous=True)
    q = current()
    target = list(q)
    target[args.joint] += math.radians(args.delta_deg)
    print(json.dumps(dict(mode='EXECUTE' if args.execute else 'PREVIEW_ONLY', joint_names=NAMES,
                          current_rad=q, target_rad=target, duration_s=args.seconds,
                          action='/rci_action_manager/jointControl', automatic_return=False), indent=2), flush=True)
    if not args.execute:
        print('Preview only: no action client or robot command created.')
        return
    import actionlib
    from actionlib_msgs.msg import GoalStatus
    from rci_action_manager.msg import JointPostureAction, JointPostureGoal, MoveitOptimalStatus
    status = rospy.wait_for_message('/real_action_controller/status', MoveitOptimalStatus, timeout=5)
    requires_ft=rospy.get_param('/real_action_controller/admittance_enabled',True)
    if (rospy.Time.now()-status.header.stamp).to_sec() > .5 or status.state != 'HOLDING' or status.reason or not status.command_enabled or (requires_ft and not status.ft_bias_ready):
        raise RuntimeError('Field action server not ready: state=%s, command_enabled=%s, ft_bias_ready=%s, age=%.3f s, reason=%r' %
                           (status.state, status.command_enabled, status.ft_bias_ready,
                            (rospy.Time.now()-status.header.stamp).to_sec(), status.reason))
    client = actionlib.SimpleActionClient('/rci_action_manager/jointControl', JointPostureAction)
    if not client.wait_for_server(rospy.Duration(5)):
        raise RuntimeError('Field action server unavailable')
    fresh = current()
    if max(abs(a-b) for a,b in zip(q, fresh)) > .005:
        raise RuntimeError('Robot moved after preview; retry at standstill')
    goal = JointPostureGoal()
    goal.target_joints.name = NAMES
    goal.target_joints.position = target
    goal.duration = args.seconds
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(180)):
        client.cancel_goal()
        raise RuntimeError('Timed out; requested cancellation of this goal')
    result = client.get_result()
    if client.get_state() != GoalStatus.SUCCEEDED or result is None or not result.is_completed:
        raise RuntimeError('Action failed: state=%s, result=%s, reason=%s' %
                           (client.get_state(), result, client.get_goal_status_text()))
    print('SUCCEEDED: small HQP joint action completed; no automatic return.')


if __name__ == '__main__':
    main()

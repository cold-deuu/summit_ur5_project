#!/usr/bin/env python3
"""Forward validated RViz plans automatically or on an explicit 'send'."""
import copy
import math
from pathlib import Path
import sys
import threading
import time

import rospy
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from rci_action_manager.msg import MoveitOptimalStatus
# Resolve sibling helpers before catkin's generated devel-space script relays.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from first_joint_check import NAMES, ordered_state


def forwarding_ready(status, now, automatic):
    ready_states = ('HOLDING', 'TRACKING') if automatic else ('HOLDING',)
    # A tolerance timeout aborts one plan but does not latch a controller fault.
    # Current state/F/T/driver/fault failures have their own blocking reasons.
    reason_ok = (status.reason in ('', 'goal tolerance satisfied', 'replaced by a newer trajectory') or
                 (status.last_result == 'ABORTED' and status.reason == 'goal tolerance timeout'))
    return (status.state in ready_states and reason_ok and status.ft_bias_ready and
            status.command_enabled and -.05 <= now-status.header.stamp.to_sec() <= .5)


def main():
    rospy.init_node('send_rviz_moveit_plan', anonymous=True)
    lock = threading.Lock()
    received = threading.Event()
    cached = [None, 0]
    used = 0
    names = rospy.get_param('/moveit_optimal_controller/joint_names', NAMES)
    if len(names) != 6 or len(set(names)) != 6:
        raise RuntimeError('Controller must specify six unique joint names.')
    state_topic = rospy.get_param('/moveit_optimal_controller/joint_state_topic', '/joint_states')
    display_topic = rospy.get_param('~display_topic', '/move_group/display_planned_path')
    auto_send = rospy.get_param('~auto_send', False)

    def receive(message):
        # A multi-stage plan needs explicit stitching; never silently drop stages.
        valid = (len(message.trajectory) == 1 and
                 not message.trajectory[0].multi_dof_joint_trajectory.points and
                 bool(message.trajectory[0].joint_trajectory.points))
        with lock:
            cached[1] += 1
            cached[0] = copy.deepcopy(message.trajectory[0].joint_trajectory) if valid else None
        if valid:
            plan = message.trajectory[0].joint_trajectory
            peak = max((abs(v) for point in plan.points for v in point.velocities), default=0.)
            print('\nPlan received: duration=%.2f s, peak joint velocity=%.4f rad/s. %s' %
                  (plan.points[-1].time_from_start.to_sec(), peak,
                   'Checking for automatic forwarding.' if auto_send else 'Type send to publish.'), flush=True)
        else:
            print('\nUnsupported/empty plan; cache cleared.', flush=True)
        received.set()

    rospy.Subscriber(display_topic, DisplayTrajectory, receive, queue_size=1)
    publisher = rospy.Publisher('/rci_action_manager/moveit_plan', JointTrajectory, queue_size=1)
    print('In RViz choose current start state, set a small goal, then Plan.\n' +
          ('Automatic forwarding ON: a valid plan is sent after readiness/start checks; Ctrl+C exits.'
           if auto_send else 'send: publish latest plan once; quit: exit. Plan alone sends no hardware goal.'), flush=True)
    while not rospy.is_shutdown():
        if auto_send:
            if not received.wait(.1):
                continue
            received.clear()
        else:
            try:
                command = input('moveit> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                return
            if command in ('quit', 'exit'):
                return
            if command != 'send':
                continue
        try:
            with lock:
                trajectory, identifier = copy.deepcopy(cached[0]), cached[1]
            if trajectory is None or identifier == used:
                raise RuntimeError('Create a new RViz plan first.')
            # Automatic mode attempts each received plan once, never later on a stale start.
            if auto_send:
                used = identifier
            if len(trajectory.joint_names) != 6 or set(trajectory.joint_names) != set(names):
                raise RuntimeError('Plan joint names must match the six controller joints.')
            status = rospy.wait_for_message('/moveit_optimal_controller/status', MoveitOptimalStatus, timeout=5)
            if not forwarding_ready(status, rospy.Time.now().to_sec(), auto_send):
                raise RuntimeError('Controller not ready: state=%s, result=%s, reason=%r, command=%s, bias=%s' %
                                   (status.state, status.last_result, status.reason,
                                    status.command_enabled, status.ft_bias_ready))
            q = ordered_state(rospy.wait_for_message(state_topic, JointState, timeout=5),
                              rospy.Time.now().to_sec(), names, require_stationary=not auto_send)
            first = trajectory.points[0].positions
            if len(first) != 6 or not all(math.isfinite(value) for value in first):
                raise RuntimeError('Invalid first point.')
            actual = dict(zip(names, q))
            error = max(abs(value - actual[name]) for name, value in zip(trajectory.joint_names, first))
            tolerance = rospy.get_param('/moveit_optimal_controller/start_tolerance', 0.05)
            if error > tolerance:
                name = max(trajectory.joint_names,
                           key=lambda n: abs(first[trajectory.joint_names.index(n)] - actual[n]))
                planned = first[trajectory.joint_names.index(name)]
                raise RuntimeError('Start mismatch: %s plan=%.4f, actual=%.4f rad, gap=%.4f rad (%.2f deg). '
                                   'In RViz set Start State to <current> and click Plan again.' %
                                   (name, planned, actual[name], error, math.degrees(error)))
            # A latched RViz plan can arrive before ROS finishes connecting the output.
            connection_deadline = time.monotonic() + 1.0
            while (not publisher.get_num_connections() and not rospy.is_shutdown() and
                   time.monotonic() < connection_deadline):
                time.sleep(.01)
            if not publisher.get_num_connections():
                raise RuntimeError('No HQP trajectory subscriber.')
            with lock:
                if auto_send and identifier != cached[1]:
                    continue  # A newer plan arrived during validation.
            trajectory.header.stamp = rospy.Time(0)
            before = max(status.trajectory_id, status.result_trajectory_id)
            publisher.publish(trajectory)
            used = identifier
            geometric = rospy.get_param('/moveit_optimal_controller/geometric_tracking', False)
            print(('Published path (MoveIt nominal duration %.2f s; geometric tracking ignores this deadline). '
                   if geometric else 'Published %.2f s plan. ') % trajectory.points[-1].time_from_start.to_sec() +
                  ('Ready for the next plan.' if auto_send else 'Waiting for controller result...'), flush=True)
            if auto_send:
                continue
            deadline = time.monotonic() + max(180., trajectory.points[-1].time_from_start.to_sec() * 10 + 30)
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                result = rospy.wait_for_message('/moveit_optimal_controller/status', MoveitOptimalStatus, timeout=5)
                if result.result_trajectory_id > before and result.last_result in ('SUCCEEDED', 'REJECTED', 'ABORTED'):
                    print('%s: %s; RMS=%.6f rad, max=%.6f rad' %
                          (result.last_result, result.reason, result.tracking_rms, result.tracking_max), flush=True)
                    break
            else:
                print('Status timeout; this does not cancel the trajectory.', flush=True)
        except (RuntimeError, ValueError, rospy.ROSException) as error:
            print(str(error), flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Cache RViz plans; publish only when the operator types 'send'."""
import copy
import math
import threading
import time

import rospy
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from rci_action_manager.msg import MoveitOptimalStatus
from first_joint_check import NAMES, ordered_state


def main():
    rospy.init_node('send_rviz_moveit_plan', anonymous=True)
    lock = threading.Lock()
    cached = [None, 0]
    used = 0

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
            print('\nPlan received: duration=%.2f s, peak joint velocity=%.4f rad/s. Type send to publish.' %
                  (plan.points[-1].time_from_start.to_sec(), peak), flush=True)
        else:
            print('\nUnsupported/empty plan; cache cleared.', flush=True)

    rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, receive, queue_size=1)
    publisher = rospy.Publisher('/rci_action_manager/moveit_plan', JointTrajectory, queue_size=1)
    print('In RViz choose current start state, set a small goal, then Plan.\n'
          'send: publish latest plan once; quit: exit. Plan alone sends no hardware goal.', flush=True)
    while not rospy.is_shutdown():
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
            if len(trajectory.joint_names) != 6 or set(trajectory.joint_names) != set(NAMES):
                raise RuntimeError('Plan must contain exactly the six native UR5e joints.')
            status = rospy.wait_for_message('/moveit_optimal_controller/status', MoveitOptimalStatus, timeout=5)
            if status.state != 'HOLDING' or not status.ft_bias_ready or not status.command_enabled:
                raise RuntimeError('Controller must be HOLDING, bias ready and command enabled.')
            q = ordered_state(rospy.wait_for_message('/joint_states', JointState, timeout=5), rospy.Time.now().to_sec())
            first = trajectory.points[0].positions
            if len(first) != 6:
                raise RuntimeError('Invalid first point.')
            actual = dict(zip(NAMES, q))
            error = max(abs(value - actual[name]) for name, value in zip(trajectory.joint_names, first))
            tolerance = rospy.get_param('/moveit_optimal_controller/start_tolerance', 0.05)
            if error > tolerance:
                name = max(trajectory.joint_names,
                           key=lambda n: abs(first[trajectory.joint_names.index(n)] - actual[n]))
                planned = first[trajectory.joint_names.index(name)]
                raise RuntimeError('Start mismatch: %s plan=%.4f, actual=%.4f rad, gap=%.4f rad (%.2f deg). '
                                   'In RViz set Start State to <current>, click Plan again, then send.' %
                                   (name, planned, actual[name], error, math.degrees(error)))
            if not publisher.get_num_connections():
                raise RuntimeError('No HQP trajectory subscriber.')
            trajectory.header.stamp = rospy.Time(0)
            before = max(status.trajectory_id, status.result_trajectory_id)
            publisher.publish(trajectory)
            used = identifier
            geometric = rospy.get_param('/moveit_optimal_controller/geometric_tracking', False)
            print(('Published path (MoveIt nominal duration %.2f s; geometric tracking ignores this deadline). '
                   if geometric else 'Published %.2f s plan. ') % trajectory.points[-1].time_from_start.to_sec() +
                  'Waiting for controller result...', flush=True)
            deadline = time.monotonic() + max(180., trajectory.points[-1].time_from_start.to_sec() * 10 + 30)
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                result = rospy.wait_for_message('/moveit_optimal_controller/status', MoveitOptimalStatus, timeout=5)
                if result.result_trajectory_id > before and result.last_result in ('SUCCEEDED', 'REJECTED', 'ABORTED'):
                    print('%s: %s; RMS=%.6f rad, max=%.6f rad' %
                          (result.last_result, result.reason, result.tracking_rms, result.tracking_max), flush=True)
                    break
            else:
                print('Status timeout; this does not cancel the trajectory.', flush=True)
        except (RuntimeError, rospy.ROSException) as error:
            print(str(error), flush=True)


if __name__ == '__main__':
    main()

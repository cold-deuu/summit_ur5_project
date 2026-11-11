#!/usr/bin/env python
"""Client half of selftest.sh. Usage: selftest.py <moveit|actions> <result.json> [reference.json]

Drives the controller started by selftest.launch, prints one PASS/FAIL line per step and writes
the measured numbers. With a reference (same test, recorded with the Noetic build) it also
reports the difference per step."""
from __future__ import division, print_function
import copy
import json
import os
import sys
import threading
import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rci_action_manager.msg import (JointPostureAction, JointPostureGoal, MoveitOptimalStatus,
                                    SE3Action, SE3Goal)

HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
         'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']
HOME = [0, -1.57, 1.57, -1.57, -1.57, 0]
POSE_A = [0.2, -1.67, 1.72, -1.47, -1.67, 0.2]
GOAL_TOL = 0.01     # rad, final joint error
REF_TOL = 0.005     # rad / m, difference to the Noetic reference

mode, result_file = sys.argv[1], sys.argv[2]
reference = {}
if len(sys.argv) > 3 and os.path.exists(sys.argv[3]):
    reference = dict((r['step'], r) for r in json.load(open(sys.argv[3])).get(mode, []))
rospy.init_node('hqp_selftest_client', anonymous=True)
status_topic = '/moveit_optimal_controller/status' if mode == 'moveit' else '/real_action_controller/status'
latest = {}
lock = threading.Lock()
history = []


def on_status(msg):
    with lock:
        latest['s'] = msg
        history.append((rospy.get_time(), msg))


rospy.Subscriber(status_topic, MoveitOptimalStatus, on_status, queue_size=200)
push_pub = rospy.Publisher('/selftest/push', WrenchStamped, queue_size=1)
results = []
failed = []


def status():
    with lock:
        return latest.get('s')


def wait(pred, timeout, what):
    end = rospy.get_time() + timeout
    while rospy.get_time() < end and not rospy.is_shutdown():
        s = status()
        if s is not None and pred(s):
            return s
        rospy.sleep(0.01)
    s = status()
    raise RuntimeError('%s: timeout after %.0f s (state=%s result=%s reason=%r)' % (
        what, timeout, s and s.state, s and s.last_result, s and s.reason))


def report(step, ok, text, **numbers):
    numbers.update(step=step, ok=bool(ok))
    ref = reference.get(step)
    if ref is not None:
        diffs = [abs(np.array(numbers[k]) - np.array(ref[k])).max() for k in ('final_q', 'moved_m') if k in numbers and k in ref]
        if diffs:
            numbers['reference_diff'] = float(max(diffs))
            text += '  | Noetic reference diff %.1e' % numbers['reference_diff']
            if numbers['reference_diff'] > REF_TOL:
                ok = False
                text += ' (> %.0e)' % REF_TOL
                numbers['ok'] = False
    results.append(numbers)
    print('[%s] %-26s %s' % ('PASS' if ok else 'FAIL', step, text))
    sys.stdout.flush()
    if not ok:
        failed.append(step)


def q_now():
    return np.array(status().actual)


def push(step, newton=6.0, seconds=0.25, settle=3.0):
    t0 = rospy.get_time()
    q0 = q_now()
    while rospy.get_time() - t0 < seconds:
        m = WrenchStamped()
        m.header.stamp = rospy.Time.now()
        m.wrench.force.x = newton
        push_pub.publish(m)
        rospy.sleep(0.02)
    rospy.sleep(settle)
    with lock:
        d = [np.linalg.norm(x.admittance_displacement[:3]) for t, x in history if t >= t0]
    excursion = float(max(d)) if d else 0.0
    shift = float(np.abs(q_now() - q0).max())
    report(step, excursion > 0.001 or shift > 0.001, 'admittance excursion %.1f mm, joint shift %.4f rad' % (1e3 * excursion, shift),
           excursion_m=excursion, joint_shift_rad=shift)


def urdf_fk(path):
    """Wrist-3 joint frame position in ur5e_base_link from the arm URDF (no pinocchio needed)."""
    import xml.etree.ElementTree as ET

    def rot(axis, a):
        x, y, z = np.array(axis) / np.linalg.norm(axis)
        c, s, C = np.cos(a), np.sin(a), 1 - np.cos(a)
        return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                         [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                         [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])
    joints = dict((j.get('name'), j) for j in ET.parse(path).getroot().iter('joint'))
    chain = []
    for n in NAMES:
        o = joints[n].find('origin')
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
        r, p, y = [float(v) for v in o.get('rpy', '0 0 0').split()]
        R = rot([0, 0, 1], y).dot(rot([0, 1, 0], p)).dot(rot([1, 0, 0], r))
        chain.append((np.array(xyz), R, [float(v) for v in joints[n].find('axis').get('xyz').split()]))

    def position(q):
        p, R = np.zeros(3), np.eye(3)
        for (xyz, Ro, axis), a in zip(chain, q):
            p = p + R.dot(xyz)
            R = R.dot(Ro).dot(rot(axis, a))
        return p
    return position


def run_moveit():
    pub = rospy.Publisher('/rci_action_manager/moveit_plan', JointTrajectory, queue_size=1)
    rospy.sleep(1.0)

    def send(traj, step, expect='SUCCEEDED'):
        s = status()
        before = max(s.trajectory_id, s.result_trajectory_id)
        pub.publish(traj)
        if expect == 'REJECTED':
            s = wait(lambda s: s.last_result == 'REJECTED' and s.result_trajectory_id > before, 5, step)
            report(step, True, 'rejected: %s' % s.reason)
            return
        s = wait(lambda s: s.trajectory_id > before, 5, step + ' (accept)')
        ident = s.trajectory_id
        s = wait(lambda s: s.result_trajectory_id == ident and s.state == 'HOLDING' and s.last_result != 'NONE', 40, step)
        rospy.sleep(0.5)
        q = q_now()
        goal = np.array([traj.points[-1].positions[traj.joint_names.index(n)] for n in NAMES])
        err = float(np.abs(q - goal).max())
        ok = s.last_result == expect and err < GOAL_TOL
        report(step, ok, '%s in %.2f s, goal error %.4f rad' % (s.last_result, s.elapsed, err),
               result=s.last_result, executed_s=round(s.elapsed, 3), goal_error_rad=err, final_q=[round(v, 5) for v in q])

    plan = json.load(open(os.path.join(HERE, 'selftest_plan.json')))
    traj = JointTrajectory()
    traj.joint_names = plan['joint_names']
    for p in plan['points']:
        pt = JointTrajectoryPoint()
        pt.positions, pt.velocities, pt.accelerations = p['positions'], p['velocities'], p['accelerations']
        pt.time_from_start = rospy.Duration(p['time_from_start']['secs'], p['time_from_start']['nsecs'])
        traj.points.append(pt)
    send(traj, 'moveit_plan_toppra')
    back = JointTrajectory()
    back.joint_names = NAMES
    q0 = q_now()
    for i in range(6):
        a = i / 5.0
        pt = JointTrajectoryPoint()
        pt.positions = list(q0 + a * (np.array(HOME) - q0))
        pt.velocities = [0.0] * 6
        pt.accelerations = [0.0] * 6
        pt.time_from_start = rospy.Duration(2.0 * a)
        back.points.append(pt)
    send(back, 'trajectory_back_home')
    send(JointTrajectory(), 'reject_empty_trajectory', expect='REJECTED')
    bad = copy.deepcopy(back)
    bad.joint_names[0] = 'unknown_joint'
    send(bad, 'reject_unknown_joint', expect='REJECTED')
    push('admittance_push_6N')


def run_actions():
    jc = actionlib.SimpleActionClient('/rci_action_manager/jointControl', JointPostureAction)
    sc = actionlib.SimpleActionClient('/rci_action_manager/se3Control', SE3Action)
    if not (jc.wait_for_server(rospy.Duration(10)) and sc.wait_for_server(rospy.Duration(10))):
        raise RuntimeError('action servers /rci_action_manager/jointControl, se3Control not found')
    position = urdf_fk(os.path.join(HERE, '..', 'field', 'ur5e_arm.urdf'))

    def joint(step, target):
        g = JointPostureGoal()
        g.duration = 4.0
        g.target_joints.name = NAMES
        g.target_joints.position = target
        jc.send_goal(g)
        done = jc.wait_for_result(rospy.Duration(30))
        rospy.sleep(0.5)
        q = q_now()
        err = float(np.abs(q - target).max())
        completed = bool(done and jc.get_result() and jc.get_result().is_completed)
        report(step, completed and err < GOAL_TOL, 'completed=%s, joint error %.1e rad' % (completed, err),
               completed=completed, joint_error_rad=err, final_q=[round(v, 5) for v in q])

    def se3(step, dx):
        p0 = position(q_now())
        g = SE3Goal()
        g.duration = 4.0
        g.relative = True
        g.wholebody = False
        g.target_pose.position.x, g.target_pose.position.y, g.target_pose.position.z = dx
        g.target_pose.orientation.w = 1.0
        sc.send_goal(g)
        done = sc.wait_for_result(rospy.Duration(30))
        rospy.sleep(0.5)
        q = q_now()
        completed = bool(done and sc.get_result() and sc.get_result().is_completed)
        moved = position(q) - p0
        ok = completed and float(np.abs(moved - np.array(dx)).max()) < 0.005
        text = 'completed=%s, wrist moved [%.3f %.3f %.3f] m (asked %s)' % ((completed,) + tuple(moved) + (list(dx),))
        extra = dict(completed=completed, moved_m=[round(v, 5) for v in moved], final_q=[round(v, 5) for v in q])
        report(step, ok, text, **extra)

    joint('joint_pose_A', POSE_A)
    joint('joint_home', HOME)
    se3('se3_translate', (0.05, 0.0, -0.03))
    se3('se3_translate_back', (-0.05, 0.0, 0.03))
    push('admittance_push_6N')
    joint('joint_home_after_push', HOME)


try:
    s = wait(lambda s: s.ft_bias_ready and s.command_enabled and s.state == 'HOLDING' and not s.reason, 40,
             'controller ready (%s)' % status_topic)
    report('controller_ready', True, 'status OK, F/T bias ready')
    rospy.sleep(1.0)
    run_moveit() if mode == 'moveit' else run_actions()
except Exception as e:
    report('aborted', False, str(e))
json.dump(dict(mode=mode, results=results, failed=failed), open(result_file, 'w'), indent=1)
print('RESULT %s %s' % (mode, 'PASS' if not failed else 'FAIL: ' + ', '.join(failed)))
sys.exit(1 if failed else 0)

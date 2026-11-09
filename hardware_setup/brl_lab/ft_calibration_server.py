#!/usr/bin/env python3
"""BRL_lab: F/T residual-gravity calibration services for the RViz panel (no robot commands).

Same checks and data format as ft_gravity.py collect/fit, but driven by buttons:
  ~new_dataset (std_srvs/Trigger): start a new dataset folder
  ~capture     (std_srvs/Trigger): record the current static pose; label from ~label param
  ~fit         (std_srvs/Trigger): fit + leave-one-out validation, write model.yaml
  ~run_all     (std_srvs/Trigger): new dataset, then for every pose: plan (move_group,
                collision checked) -> HQP -> settle -> capture; finally fit. Background thread.
  ~stop        (std_srvs/Trigger): stop the automatic run and hold the current position
  ~progress    (std_msgs/String topic): "RUNNING ...", "DONE ...", "STOPPED ..." for the panel
Requires the HQP controller running with admittance_enabled:=false and HOLDING.
Only ~run_all moves the robot (through HQP); everything else is read-only.
"""
from collections import deque
import datetime
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rospy
import tf
import yaml
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rci_action_manager.msg import ControlTelemetry, MoveitOptimalStatus

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import ft_gravity  # noqa: E402  (hardware_setup/ft_gravity.py: checks, helpers)
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ft_w3_model  # noqa: E402  (gravity + wrist_3 offset model)


# Same poses as the panel's F/T calibration rows (brl_lab_rviz_panel kPoses), degrees.
POSES = [
    ('C1', [60, -90, -90, -90, 90, 0]), ('C2', [60, -90, -90, -90, 0, 0]),
    ('C3', [60, -90, -90, -45, 0, 0]), ('C4', [60, -90, -90, 0, 0, 0]),
    ('C5', [60, -90, -90, 45, 0, 0]), ('C6', [60, -90, -90, 0, 0, 90]),
    ('C7', [60, -90, -90, -45, -90, 0]), ('C8', [60, -90, -90, -90, -90, 0]),
    ('C9', [60, -90, -90, -135, 0, 0]), ('C10', [60, -90, -90, -90, 0, -90]),
    ('S1', [60, -90, -90, -90, 90, -180]), ('S2', [60, -90, -90, -90, 90, -135]),
    ('S3', [60, -90, -90, -90, 90, -90]), ('S4', [60, -90, -90, -90, 90, -45]),
    ('S5', [60, -90, -90, -90, 90, 0]), ('S6', [60, -90, -90, -90, 90, 45]),
    ('S7', [60, -90, -90, -90, 90, 90]), ('S8', [60, -90, -90, -90, 90, 135]),
    ('S9', [60, -90, -90, -90, 90, 180]),
]


class StopRun(Exception):
    pass


class CalibrationServer:
    def __init__(self):
        self.controller = rospy.get_param('~controller', '/moveit_optimal_controller').rstrip('/')
        self.input = rospy.get_param('~input', '/hqp_moveit/wrench')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.wrench_frame = rospy.get_param('~wrench_frame', 'wrist_3_link')
        self.gravity = [0.0, 0.0, -9.80665]
        self.seconds = float(rospy.get_param('~seconds', 4.0))
        self.min_samples = int(rospy.get_param('~min_samples', 20))
        self.root = Path(rospy.get_param('~output_root', '~/catkin_ws/artifacts/ft_gravity')).expanduser()
        self.listener = tf.TransformListener()
        self.lock = threading.Lock()
        self.pending = deque(maxlen=500)
        self.states = deque(maxlen=1000)
        self.telemetry = None
        self.dataset_path = None
        self.data = None
        rospy.Subscriber(self.input, WrenchStamped, self._wrench, queue_size=100)
        rospy.Subscriber(self.controller + '/telemetry', ControlTelemetry, self._telemetry, queue_size=10)
        self.state_topic = None
        self.status_msg = None
        self.status_stamp = 0.0
        rospy.Subscriber(self.controller + '/status', MoveitOptimalStatus, self._status, queue_size=10)
        self.plan_pub = rospy.Publisher('/rci_action_manager/moveit_plan', JointTrajectory, queue_size=1)
        self.progress_pub = rospy.Publisher('~progress', String, queue_size=10, latch=True)
        self.run_thread = None
        self.stop_requested = threading.Event()
        self.settle = float(rospy.get_param('~settle_seconds', 2.0))
        rospy.Service('~run_all', Trigger, self.run_all)
        rospy.Service('~stop', Trigger, self.stop)
        rospy.Service('~new_dataset', Trigger, self.new_dataset)
        rospy.Service('~capture', Trigger, self.capture)
        rospy.Service('~fit', Trigger, self.fit)
        rospy.Service('~status', Trigger, self.status)
        # Resume the latest dataset (e.g. after an RViz restart). Captures must come from the same
        # UR program run (the UR re-zeroes the F/T at program start); use New dataset otherwise.
        latest = sorted(self.root.glob('*/poses.json')) if rospy.get_param('~resume_latest', True) else []
        if latest:
            self.dataset_path = latest[-1]
            self.data = json.loads(self.dataset_path.read_text())
            rospy.loginfo('Resumed F/T dataset %s (%d poses)', self.dataset_path, len(self.data['poses']))
        else:
            self.new_dataset(None)

    def _wrench(self, message):
        with self.lock:
            self.pending.append(message)

    def _state(self, message):
        with self.lock:
            self.states.append(message)

    def _telemetry(self, message):
        with self.lock:
            self.telemetry = message

    def _status(self, message):
        with self.lock:
            self.status_msg = message
            self.status_stamp = time.monotonic()

    def new_dataset(self, _request):
        folder = self.root / datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.dataset_path = folder / 'poses.json'
        self.data = dict(schema=1, base_frame=self.base_frame, wrench_frame=self.wrench_frame,
                         gravity_base=self.gravity, wrench_topic=self.input, poses=[],
                         note='BRL_lab panel calibration: unloaded stationary poses; UR payload unchanged.')
        return TriggerResponse(success=True, message='New dataset: %s' % folder)

    def status(self, _request):
        return TriggerResponse(success=True, message='Dataset %s: %d poses (%s)' % (
            self.dataset_path, len(self.data['poses']), ', '.join(p['label'] for p in self.data['poses'])))

    def _check_controller(self):
        if rospy.get_param(self.controller + '/admittance_enabled', True):
            raise ValueError('Admittance is ON. Restart HQP with admittance_enabled:=false')
        names = rospy.get_param(self.controller + '/joint_names')
        if self.state_topic is None:
            self.state_topic = rospy.get_param(self.controller + '/joint_state_topic', '/joint_states')
            rospy.Subscriber(self.state_topic, JointState, self._state, queue_size=100)
            rospy.sleep(0.3)
        return names

    def capture(self, _request):
        return self.capture_label(str(rospy.get_param('~label', '')).strip())

    def capture_label(self, label):
        try:
            names = self._check_controller()
            if not label:
                raise ValueError('No pose label')
            existing = {p['label'] for p in self.data['poses']}
            base, suffix = label, 2
            while label in existing:
                label = '%s_%d' % (base, suffix)
                suffix += 1
            with self.lock:
                self.pending.clear()
            samples, joint_samples, gravities, last_stamp = [], [], [], 0.0
            deadline = time.monotonic() + self.seconds
            while time.monotonic() < deadline and not rospy.is_shutdown():
                with self.lock:
                    message = self.pending.popleft() if self.pending else None
                    joints = list(self.states)
                    telemetry = self.telemetry
                if message is None:
                    time.sleep(0.005)
                    continue
                now = rospy.Time.now().to_sec()
                stamp = message.header.stamp.to_sec()
                if stamp <= last_stamp or not 0 <= now - stamp <= 0.2:
                    continue
                if message.header.frame_id.lstrip('/') != self.wrench_frame:
                    raise ValueError('Unexpected wrench frame %s' % message.header.frame_id)
                if telemetry is None or not 0 <= now - telemetry.header.stamp.to_sec() < 0.3:
                    continue
                if telemetry.mode != 'HOLDING' or telemetry.reason or telemetry.fault_latched or not telemetry.ft_bias_ready:
                    raise ValueError('Controller must be healthy HOLDING with F/T bias ready (wait for the move to finish)')
                if not joints:
                    continue
                joint = min(joints, key=lambda j: abs(j.header.stamp.to_sec() - stamp))
                if abs(joint.header.stamp.to_sec() - stamp) > 0.05:
                    continue
                indices = [joint.name.index(name) for name in names]
                q = np.array([joint.position[i] for i in indices])
                v = np.array([joint.velocity[i] for i in indices])
                if np.max(np.abs(v)) >= 0.01:
                    raise ValueError('Robot is moving; wait until it stops')
                try:
                    _, quaternion = self.listener.lookupTransform(self.base_frame, self.wrench_frame, message.header.stamp)
                except tf.Exception:
                    continue
                samples.append(ft_gravity.wrench_value(message))
                joint_samples.append(q)
                gravities.append(ft_gravity.rotation(quaternion).T @ np.array(self.gravity))
                last_stamp = stamp
            if len(samples) < self.min_samples:
                raise ValueError('Too few fresh samples (%d); is HQP publishing %s?' % (len(samples), self.input))
            if np.max(np.ptp(joint_samples, axis=0)) > 0.003:
                raise ValueError('Joint posture drifted during capture; retry')
            spread = np.std(samples, axis=0)
            if max(spread[:3]) > 1 or max(spread[3:]) > 0.08:
                raise ValueError('Unstable wrench (std %.2f N / %.3f Nm); keep the tool untouched and retry'
                                 % (max(spread[:3]), max(spread[3:])))
            mean = np.mean(samples, axis=0)
            self.data['poses'].append(dict(label=label, samples=len(samples), wrench=mean.tolist(),
                                           wrench_std=spread.tolist(),
                                           gravity_wrist=np.mean(gravities, axis=0).tolist(),
                                           joints=np.mean(joint_samples, axis=0).tolist()))
            self.dataset_path.parent.mkdir(parents=True, exist_ok=True)
            self.dataset_path.write_text(json.dumps(self.data, indent=2))
            return TriggerResponse(success=True, message='%s: |F| %.2f N |T| %.3f Nm (%d samples, %d poses)'
                                   % (label, np.linalg.norm(mean[:3]), np.linalg.norm(mean[3:]),
                                      len(samples), len(self.data['poses'])))
        except (ValueError, KeyError, IndexError) as error:
            return TriggerResponse(success=False, message=str(error))

    def fit(self, _request):
        try:
            if rospy.get_param(self.controller + '/admittance_enabled', True):
                raise ValueError('Admittance is ON. Restart HQP with admittance_enabled:=false')
            model = ft_w3_model.fit(self.data, 1.0, 0.08)
            path = self.dataset_path.parent / 'model.yaml'
            path.write_text(yaml.safe_dump(model, sort_keys=False))
            # HQP launches default to <root>/current/model.yaml: point it at this validated model.
            current = self.root / 'current'
            temporary = self.root / '.current_tmp'
            if temporary.is_symlink() or temporary.exists():
                temporary.unlink()
            temporary.symlink_to(self.dataset_path.parent)
            temporary.replace(current)
            loo = model['validation']['leave_one_out']
            worst = max(loo['per_pose'], key=lambda p: p['force_N'])
            cog = model.get('effective_cog_m')
            return TriggerResponse(success=True, message=(
                'Model OK (gravity + wrist_3 offset, K=%d): %s\nmass %.3f kg, CoG %s m, bias F %s N\n'
                'leave-one-out RMS %.2f N / %.3f Nm (worst %s %.2f N)\n'
                'Now the default for HQP (artifacts/ft_gravity/current). Restart HQP to apply: '
                'roslaunch summit_ur5_hardware_setup ur5e_moveit_brl_lab.launch (%s)'
                % (model['order'], path, model['effective_mass_kg'], None if cog is None else [round(c, 3) for c in cog],
                   [round(b, 2) for b in model['bias'][:3]], loo['force_rms_N'], loo['torque_rms_Nm'],
                   worst['label'], worst['force_N'], path)))
        except (ValueError, KeyError) as error:
            return TriggerResponse(success=False, message='Fit rejected: %s' % error)


    # ---------------- automatic run (moves the robot through HQP) ----------------
    def _progress(self, text):
        rospy.loginfo('[F/T auto] %s', text)
        self.progress_pub.publish(String(data=text))

    def _hqp(self):
        with self.lock:
            status, age = self.status_msg, time.monotonic() - self.status_stamp
        if status is None or age > 0.5:
            raise StopRun('No fresh HQP status')
        return status

    def _ready(self):
        status = self._hqp()
        reason_ok = status.reason in ('', 'goal tolerance satisfied', 'replaced by a newer trajectory') or \
            (status.last_result == 'ABORTED' and status.reason == 'goal tolerance timeout')
        if status.state != 'HOLDING' or not reason_ok or not status.command_enabled or not status.ft_bias_ready:
            raise StopRun('HQP not ready: state=%s reason=%r command=%s bias=%s'
                          % (status.state, status.reason, status.command_enabled, status.ft_bias_ready))
        return status

    def _hold(self):
        with self.lock:
            status = self.status_msg
        names = rospy.get_param(self.controller + '/joint_names', [])
        if status is None or len(names) != 6:
            return
        hold = JointTrajectory(joint_names=list(names))
        for t in (0.0, 0.5):
            hold.points.append(JointTrajectoryPoint(positions=list(status.actual), velocities=[0.0] * 6,
                                                    accelerations=[0.0] * 6, time_from_start=rospy.Duration(t)))
        self.plan_pub.publish(hold)

    def _move(self, label, goal_deg, names):
        goal = [np.radians(v) for v in goal_deg]
        with self.lock:
            joints = self.states[-1] if self.states else None
        if joints is None:
            raise StopRun('No joint states')
        current = [joints.position[joints.name.index(n)] for n in names]
        if max(abs(a - b) for a, b in zip(current, goal)) < np.radians(0.5):
            return  # already there
        request = GetMotionPlanRequest()
        mp = request.motion_plan_request
        mp.group_name = 'manipulator'
        mp.start_state.is_diff = True
        mp.num_planning_attempts = 5
        mp.allowed_planning_time = 5.0
        mp.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=p, tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for n, p in zip(names, goal)])]
        planner = rospy.ServiceProxy('/plan_kinematic_path', GetMotionPlan)
        planner.wait_for_service(5.0)
        response = planner(request).motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.trajectory.joint_trajectory.points) < 2:
            raise StopRun('%s: planning failed (move_group error %d)' % (label, response.error_code.val))
        trajectory = response.trajectory.joint_trajectory
        before_status = self._ready()
        before = max(before_status.trajectory_id, before_status.result_trajectory_id)
        if self.stop_requested.is_set():
            raise StopRun('Stopped by user')
        trajectory.header.stamp = rospy.Time(0)
        self.plan_pub.publish(trajectory)
        sent = time.monotonic()
        while not rospy.is_shutdown():
            if self.stop_requested.is_set():
                self._hold()
                raise StopRun('Stopped by user (hold sent)')
            status = self._hqp()
            if status.result_trajectory_id > before and status.last_result != 'NONE':
                if status.last_result != 'SUCCEEDED':
                    raise StopRun('%s: HQP %s - %s' % (label, status.last_result, status.reason))
                return
            if status.trajectory_id <= before and time.monotonic() - sent > 3.0:
                raise StopRun('%s: trajectory not accepted by HQP (%s)' % (label, status.reason))
            if time.monotonic() - sent > 120.0:
                self._hold()
                raise StopRun('%s: motion timeout (hold sent)' % label)
            time.sleep(0.05)

    def _run(self):
        try:  # pre-checks: nothing has been sent to the robot yet, so no hold
            names = self._check_controller()
            self._ready()
        except (StopRun, ValueError, KeyError) as error:
            self._progress('STOPPED before moving: %s' % error)
            return
        try:
            folder = self.new_dataset(None).message
            self._progress('RUNNING 0/%d: %s' % (len(POSES), folder))
            for index, (label, goal) in enumerate(POSES):
                if self.stop_requested.is_set():
                    raise StopRun('Stopped by user')
                self._progress('RUNNING %d/%d %s: moving' % (index + 1, len(POSES), label))
                self._move(label, goal, names)
                self._progress('RUNNING %d/%d %s: settling / recording' % (index + 1, len(POSES), label))
                for attempt in range(2):
                    time.sleep(self.settle if attempt == 0 else self.settle * 2)
                    if self.stop_requested.is_set():
                        raise StopRun('Stopped by user')
                    if rospy.get_param(self.controller + '/admittance_enabled', True):
                        raise StopRun('Admittance turned ON during the run')
                    result = self.capture_label(label)
                    if result.success:
                        break
                else:
                    raise StopRun('%s: capture failed twice - %s' % (label, result.message))
                self._progress('RUNNING %d/%d %s: %s' % (index + 1, len(POSES), label, result.message))
            self._progress('RUNNING fitting model')
            result = self.fit(None)
            self._progress(('DONE ' if result.success else 'FIT FAILED ') + result.message)
        except StopRun as error:
            self._progress('STOPPED %s (dataset kept: %s)' % (error, self.dataset_path))
        except (ValueError, KeyError, rospy.ROSException, rospy.ServiceException) as error:
            self._hold()
            self._progress('STOPPED error: %s (hold sent)' % error)

    def run_all(self, _request):
        if self.run_thread is not None and self.run_thread.is_alive():
            return TriggerResponse(success=False, message='Automatic calibration already running')
        self.stop_requested.clear()
        self.run_thread = threading.Thread(target=self._run, daemon=True)
        self.run_thread.start()
        return TriggerResponse(success=True, message='Automatic calibration started (%d poses)' % len(POSES))

    def stop(self, _request):
        self.stop_requested.set()
        self._hold()
        return TriggerResponse(success=True, message='Stop requested; hold sent')


if __name__ == '__main__':
    rospy.init_node('ft_calibration')
    CalibrationServer()
    rospy.spin()

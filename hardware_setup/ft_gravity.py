#!/usr/bin/env python3
"""Inspect URDF payloads, collect unloaded poses, fit and subtract residual F/T gravity."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('Expected %d finite values' % size)
    return result


def rotation(quaternion):
    x, y, z, w = vector(quaternion, 4)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-8:
        raise ValueError('Invalid orientation')
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def skew(value):
    x, y, z = value
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def design(poses):
    gravity = [vector(p['gravity_wrist'], 3) for p in poses]
    return (np.vstack([np.column_stack([g, np.eye(3)]) for g in gravity]),
            np.vstack([np.column_stack([-skew(g), np.eye(3)]) for g in gravity]))


def solve(poses):
    force, torque = design(poses)
    observations = np.array([vector(p['wrench'], 6) for p in poses])
    conditions = []
    for matrix in [force, torque]:
        norms = np.linalg.norm(matrix, axis=0)
        if np.any(norms < 1e-10):
            raise ValueError('Insufficient orientation diversity; vary tilt about more than one axis')
        singular = np.linalg.svd(matrix / norms, compute_uv=False)
        condition = singular[0] / max(singular[-1], 1e-16)
        if condition > 100:
            raise ValueError('Insufficient orientation diversity (condition %.1f)' % condition)
        conditions.append(float(condition))
    f = np.linalg.lstsq(force, observations[:, :3].reshape(-1), rcond=None)[0]
    t = np.linalg.lstsq(torque, observations[:, 3:].reshape(-1), rcond=None)[0]
    return dict(effective_mass_kg=float(f[0]), first_moment_kg_m=t[:3].tolist(),
                bias=np.r_[f[1:], t[3:]].tolist(), conditions=conditions)


def predict(model, gravity_wrist):
    g = vector(gravity_wrist, 3)
    return np.r_[float(model['effective_mass_kg']) * g,
                 np.cross(vector(model['first_moment_kg_m'], 3), g)] + vector(model['bias'], 6)


def errors(model, poses):
    residual = np.array([vector(p['wrench'], 6) - predict(model, p['gravity_wrist']) for p in poses])
    return dict(force_rms_N=float(np.sqrt(np.mean(np.sum(residual[:, :3]**2, axis=1)))),
                torque_rms_Nm=float(np.sqrt(np.mean(np.sum(residual[:, 3:]**2, axis=1)))),
                per_pose=[dict(label=p['label'], force_N=float(np.linalg.norm(e[:3])),
                               torque_Nm=float(np.linalg.norm(e[3:]))) for p, e in zip(poses, residual)])


def fit(dataset, max_force, max_torque):
    if not all(math.isfinite(v) and v > 0 for v in [max_force, max_torque]):
        raise ValueError('Validation thresholds must be finite and positive')
    poses = dataset['poses']
    if len(poses) < 6:
        raise ValueError('Collect at least six distinct unloaded orientations')
    model = solve(poses)
    held_out = []
    for i, pose in enumerate(poses):
        other = solve(poses[:i] + poses[i+1:])
        held_out.append(dict(label=pose['label'], gravity_wrist=pose['gravity_wrist'],
                             wrench=(vector(pose['wrench'], 6)-predict(other, pose['gravity_wrist'])).tolist()))
    report = errors(dict(effective_mass_kg=0, first_moment_kg_m=[0]*3, bias=[0]*6), held_out)
    if report['force_rms_N'] > max_force or report['torque_rms_Nm'] > max_torque:
        raise ValueError('Held-out residual too large: %.3f N, %.4f Nm; check contact, model or pose diversity' %
                         (report['force_rms_N'], report['torque_rms_Nm']))
    model.update(schema=1, base_frame=dataset['base_frame'], wrench_frame=dataset['wrench_frame'],
                 gravity_base=dataset['gravity_base'], validation=dict(accepted=True, leave_one_out=report),
                 interpretation='Residual gravity in the existing wrench stream; not a UR payload setting.')
    model['effective_cog_m'] = (np.array(model['first_moment_kg_m'])/model['effective_mass_kg']).tolist() if abs(model['effective_mass_kg']) > .05 else None
    return model


def load_model(path):
    model = yaml.safe_load(Path(path).read_text())
    if model.get('schema') != 1 or model.get('validation', {}).get('accepted') is not True:
        raise ValueError('Model has not passed calibration validation')
    if not math.isfinite(float(model['effective_mass_kg'])):
        raise ValueError('Nonfinite mass coefficient')
    vector(model['first_moment_kg_m'], 3); vector(model['bias'], 6)
    if np.linalg.norm(vector(model['gravity_base'], 3)) < 1:
        raise ValueError('Invalid gravity vector')
    if not model['base_frame'] or not model['wrench_frame']:
        raise ValueError('Frame names required')
    return model


def wrench_value(message):
    w = message.wrench
    return vector([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], 6)


def capture(args):
    import rospy
    import tf
    from geometry_msgs.msg import WrenchStamped
    from sensor_msgs.msg import JointState
    from rci_action_manager.msg import ControlTelemetry
    rospy.init_node('collect_ft_gravity', anonymous=True)
    vector(args.gravity, 3)
    ns = args.controller.rstrip('/')
    if rospy.get_param(ns+'/admittance_enabled', True):
        raise ValueError('Restart the controller with admittance_enabled:=false for collection')
    names = rospy.get_param(ns+'/joint_names')
    state_topic = rospy.get_param(ns+'/joint_state_topic')
    if len(names) != 6 or len(set(names)) != 6:
        raise ValueError('Six unique controller joint names required')
    data = dict(schema=1, base_frame=args.base_frame, wrench_frame=args.wrench_frame,
                gravity_base=args.gravity, wrench_topic=args.input, poses=[],
                note='Operator-declared unloaded stationary poses; keep UR payload/zero settings unchanged.')
    output = Path(args.output).expanduser()
    if output.exists():
        raise ValueError('Output already exists; choose a new dataset path')
    listener = tf.TransformListener()
    lock = threading.Lock(); pending = deque(maxlen=500); states = deque(maxlen=1000); telemetry = [None]
    def receive_wrench(msg):
        with lock: pending.append(msg)
    def receive_state(msg):
        with lock: states.append(msg)
    def receive_telemetry(msg):
        with lock: telemetry[0] = msg
    subscribers = [rospy.Subscriber(args.input, WrenchStamped, receive_wrench, queue_size=100),
                   rospy.Subscriber(state_topic, JointState, receive_state, queue_size=100),
                   rospy.Subscriber(ns+'/telemetry', ControlTelemetry, receive_telemetry, queue_size=10)]
    print('No robot commands or zeroing. Keep admittance OFF and all payload settings unchanged.\n'
          'Move to an unloaded pose yourself, wait for motion to settle, then enter a label.\n'
          'Use at least six diverse tilts about two axes. A return-to-first pose is useful.\n'
          'Each label declares no tool contact. Enter quit to finish.', flush=True)
    while not rospy.is_shutdown():
        try: label = input('unloaded pose label> ').strip()
        except (EOFError, KeyboardInterrupt): break
        if label in ['quit', 'exit']: break
        if not label: continue
        try:
            if any(p['label'] == label for p in data['poses']):
                raise ValueError('Use a unique label')
            with lock: pending.clear()
            samples = []; joint_samples = []; gravities = []; last_stamp = 0
            deadline = time.monotonic()+args.seconds
            while time.monotonic() < deadline and not rospy.is_shutdown():
                with lock:
                    msg = pending.popleft() if pending else None
                    joints = list(states); tel = telemetry[0]
                if msg is None: time.sleep(.005); continue
                now = rospy.Time.now().to_sec(); stamp = msg.header.stamp.to_sec()
                if stamp <= last_stamp or not 0 <= now-stamp <= .2: continue
                if msg.header.frame_id.lstrip('/') != args.wrench_frame.lstrip('/'):
                    raise ValueError('Unexpected wrench frame')
                if tel is None or not 0 <= now-tel.header.stamp.to_sec() < .3: continue
                if tel.mode != 'HOLDING' or tel.reason or tel.fault_latched or not tel.ft_bias_ready:
                    raise ValueError('Controller must be healthy HOLDING with bias ready')
                if not joints: continue
                joint = min(joints, key=lambda j:abs(j.header.stamp.to_sec()-stamp))
                if abs(joint.header.stamp.to_sec()-stamp) > .05: continue
                indices = [joint.name.index(name) for name in names]
                if len(joint.velocity) != len(joint.name): raise ValueError('Measured velocities required')
                q = vector([joint.position[i] for i in indices], 6)
                v = vector([joint.velocity[i] for i in indices], 6)
                if np.max(np.abs(v)) >= .01: raise ValueError('Robot moved during collection; retry pose')
                try: _, quaternion = listener.lookupTransform(args.base_frame, args.wrench_frame, msg.header.stamp)
                except tf.Exception: continue
                samples.append(wrench_value(msg)); joint_samples.append(q)
                gravities.append(rotation(quaternion).T @ vector(args.gravity, 3)); last_stamp = stamp
            if rospy.get_param(ns+'/admittance_enabled', True): raise ValueError('Admittance enabled during capture')
            if len(samples) < args.min_samples: raise ValueError('Too few fresh synchronized samples: %d' % len(samples))
            if np.max(np.ptp(joint_samples, axis=0)) > .003: raise ValueError('Joint posture drifted; retry pose')
            spread = np.std(samples, axis=0)
            if max(spread[:3]) > 1 or max(spread[3:]) > .08: raise ValueError('Unstable wrench; wait and retry')
            data['poses'].append(dict(label=label, samples=len(samples), wrench=np.mean(samples, axis=0).tolist(),
                                      wrench_std=spread.tolist(), gravity_wrist=np.mean(gravities, axis=0).tolist(),
                                      joints=np.mean(joint_samples, axis=0).tolist()))
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(data, indent=2))
            print('Saved %s: %d samples; %d poses total' % (label, len(samples), len(data['poses'])), flush=True)
        except (ValueError, IndexError) as exc: print('Rejected pose:', exc, flush=True)
    print('Dataset:', output, flush=True)


def run(args):
    import rospy
    import tf
    from geometry_msgs.msg import WrenchStamped
    model = load_model(Path(args.model).expanduser())
    rospy.init_node('ft_gravity_compensator')
    if rospy.resolve_name(args.input) == rospy.resolve_name(args.output):
        raise ValueError('Input and output topics must differ')
    listener = tf.TransformListener(); lock = threading.Lock(); queue = deque(maxlen=100)
    publisher = rospy.Publisher(args.output, WrenchStamped, queue_size=5)
    def receive(msg):
        if msg.header.frame_id.lstrip('/') != model['wrench_frame'].lstrip('/'):
            rospy.logwarn_throttle(2, 'Gravity compensation: unexpected wrench frame'); return
        if msg.header.stamp.is_zero(): return
        try: wrench_value(msg)
        except ValueError: return
        with lock: queue.append(msg)
    subscriber = rospy.Subscriber(args.input, WrenchStamped, receive, queue_size=20, tcp_nodelay=True)
    rospy.loginfo('Residual F/T gravity compensation %s -> %s; effective mass %.4f kg',
                  args.input, args.output, model['effective_mass_kg'])
    last_stamp = 0
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        with lock: candidates = list(reversed(queue))
        for msg in candidates:
            stamp = msg.header.stamp.to_sec()
            if stamp <= last_stamp or not 0 <= now-stamp <= .2: continue
            try:
                _, quaternion = listener.lookupTransform(model['base_frame'], model['wrench_frame'], msg.header.stamp)
            except tf.Exception: continue
            gravity = rotation(quaternion).T @ vector(model['gravity_base'], 3)
            value = wrench_value(msg)-predict(model, gravity)
            out = WrenchStamped(); out.header = msg.header
            out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = value[:3]
            out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = value[3:]
            publisher.publish(out); last_stamp = stamp
            with lock:
                while queue and queue[0].header.stamp.to_sec() <= stamp: queue.popleft()
            break
        else:
            rospy.logwarn_throttle(3, 'Waiting for fresh wrench and exact-stamp wrist orientation; withholding output')
        time.sleep(.01)


def inspect_urdf(args):
    if args.urdf:
        text = Path(args.urdf).expanduser().read_text()
    else:
        import rospy
        text = rospy.get_param(args.param)
    root = ET.fromstring(text); links = {l.get('name'): l for l in root.findall('link')}
    if args.root not in links: raise ValueError('Missing root link '+args.root)
    selected = {args.root}
    while True:
        children = {j.find('child').get('link') for j in root.findall('joint') if j.find('parent').get('link') in selected}
        if children <= selected: break
        selected |= children
    known = []; missing = []
    for name in sorted(selected-{args.root}):
        inertial = links[name].find('inertial')
        if inertial is None: missing.append(name); continue
        known.append(dict(link=name, mass_kg=float(inertial.find('mass').get('value')),
                          local_com=inertial.find('origin').get('xyz', '0 0 0') if inertial.find('origin') is not None else '0 0 0'))
    print(json.dumps(dict(known_mass_kg=sum(k['mass_kg'] for k in known), links=known,
                          missing_inertials=missing, note='Incomplete model sum; not a measured tool payload. CoM entries are in each link frame.'), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    p = sub.add_parser('inspect-urdf'); p.add_argument('--urdf'); p.add_argument('--param', default='/brl_robot/robot_description')
    p.add_argument('--root', default='ur5e_wrist_3_link')
    p = sub.add_parser('collect'); p.add_argument('--output', required=True)
    p.add_argument('--controller', default='/moveit_optimal_controller'); p.add_argument('--input', default='/hqp_moveit/wrench')
    p.add_argument('--base-frame', default='ur5e_base_link'); p.add_argument('--wrench-frame', default='ur5e_wrist_3_link')
    p.add_argument('--gravity', nargs=3, type=float, default=[0, 0, -9.80665])
    p.add_argument('--seconds', type=float, default=4); p.add_argument('--min-samples', type=int, default=20)
    for name in ['fit', 'validate']:
        p = sub.add_parser(name); p.add_argument('dataset'); p.add_argument('--model', required=True)
        p.add_argument('--max-force-rms', type=float, default=1.); p.add_argument('--max-torque-rms', type=float, default=.08)
    p = sub.add_parser('run'); p.add_argument('--model', required=True)
    p.add_argument('--input', default='/hqp_moveit/wrench'); p.add_argument('--output', default='/hqp_moveit/wrench_compensated')
    # Permit standard ROS remappings/private arguments when launched with rosrun/roslaunch.
    argv = [a for a in sys.argv[1:] if ':=' not in a]
    args = parser.parse_args(argv)
    if args.mode == 'inspect-urdf': inspect_urdf(args)
    elif args.mode == 'collect':
        if not 1 <= args.seconds <= 30 or args.min_samples < 10: raise ValueError('Invalid capture duration/sample count')
        capture(args)
    elif args.mode == 'run': run(args)
    else:
        dataset = json.loads(Path(args.dataset).expanduser().read_text())
        path = Path(args.model).expanduser()
        if args.mode == 'fit':
            model = fit(dataset, args.max_force_rms, args.max_torque_rms)
            path.parent.mkdir(parents=True, exist_ok=True); path.write_text(yaml.safe_dump(model, sort_keys=False))
            print(yaml.safe_dump(model, sort_keys=False))
        else:
            if not all(math.isfinite(v) and v > 0 for v in [args.max_force_rms, args.max_torque_rms]):
                raise ValueError('Validation thresholds must be finite and positive')
            model = load_model(path)
            if any(dataset[k] != model[k] for k in ['base_frame', 'wrench_frame', 'gravity_base']):
                raise ValueError('Calibration and validation frames/gravity differ')
            report = errors(model, dataset['poses']); print(json.dumps(report, indent=2))
            if report['force_rms_N'] > args.max_force_rms or report['torque_rms_Nm'] > args.max_torque_rms:
                raise ValueError('Independent validation failed')


if __name__ == '__main__':
    try: main()
    except (ValueError, KeyError) as exc:
        print('ERROR:', exc, file=sys.stderr); sys.exit(2)

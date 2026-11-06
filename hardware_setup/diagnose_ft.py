#!/usr/bin/env python3
"""Read-only stationary wrench snapshots. No publishers or service calls."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from rci_action_manager.msg import ControlTelemetry

NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def wrench(message):
    w = message.wrench
    return [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]


def summarize(rows):
    result = {}
    for prefix in ('raw', 'wrist', 'corrected', 'filtered'):
        data = np.array([[row[prefix + '_' + str(i)] for i in range(6)] for row in rows])
        result[prefix] = dict(mean=data.mean(axis=0).tolist(), std=data.std(axis=0).tolist())
    q = np.array([[row['q_' + str(i)] for i in range(6)] for row in rows])
    result['joint_range_rad'] = np.ptp(q, axis=0).tolist()
    return result


def plot(out, poses):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    good = [p for p in poses if p['accepted']]
    if not good:
        return
    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    for i, ax in enumerate(axes.flat):
        for prefix, shift in [('raw', -.24), ('wrist', -.08), ('corrected', .08), ('filtered', .24)]:
            ax.errorbar(np.arange(len(good)) + shift,
                        [p['statistics'][prefix]['mean'][i] for p in good],
                        yerr=[p['statistics'][prefix]['std'][i] for p in good], fmt='o', label=prefix)
        ax.set_xticks(np.arange(len(good)))
        ax.set_xticklabels([p['label'] for p in good], rotation=25)
        ax.set_title(('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz')[i])
        ax.set_ylabel('N' if i < 3 else 'Nm')
        ax.grid(True)
    axes[0, 0].legend()
    fig.tight_layout()
    fig.savefig(str(out / 'wrench_by_pose.png'), dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    if not 1 <= args.seconds <= 30:
        parser.error('--seconds must be between 1 and 30')
    rospy.init_node('diagnose_ft', anonymous=True)
    ns = '/moveit_optimal_controller'
    if rospy.get_param(ns + '/admittance_enabled', True):
        raise RuntimeError('Restart ur5e_moveit_real.launch admittance_enabled:=false first.')
    out = args.output or Path.home() / 'catkin_ws/artifacts/ft_diagnostics' / datetime.now().strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    (out / 'metadata.json').write_text(json.dumps(dict(
        parameters=rospy.get_param(ns, {}), units='force N, torque Nm, joints rad',
        note='Stationary snapshots; source stamps retained. Frames must be checked before comparing components.'), indent=2))
    latest = {}
    lock = threading.Lock()

    def receive(name, message):
        with lock:
            latest[name] = message

    subscriptions = [rospy.Subscriber(topic, kind, lambda msg, key=name: receive(key, msg), queue_size=5)
                     for name, topic, kind in [
                         ('raw', '/wrench', WrenchStamped),
                         ('wrist', '/hqp_moveit/wrench', WrenchStamped),
                         ('telemetry', ns + '/telemetry', ControlTelemetry),
                         ('joints', '/joint_states', JointState)]]
    poses = []
    print('Read-only. No robot commands, zeroing or parameter changes.\n'
          'At each untouched stationary pose, enter a label (A, B, C...).\n'
          'Wait 2 seconds after motion. Enter quit when finished.\nOutput: ' + str(out), flush=True)
    try:
        while not rospy.is_shutdown():
            label = input('pose label> ').strip()
            if label.lower() in ('quit', 'exit', 'q'):
                break
            if not label:
                continue
            rows = []
            last_stamp = None
            deadline = time.monotonic() + args.seconds
            error = ''
            while time.monotonic() < deadline and not rospy.is_shutdown():
                if rospy.get_param(ns + '/admittance_enabled', True):
                    raise RuntimeError('Admittance enabled during diagnosis; stop capture.')
                with lock:
                    messages = dict(latest)
                if len(messages) != 4:
                    error = 'Missing topic(s): ' + ', '.join(sorted(set(['raw', 'wrist', 'telemetry', 'joints']) - set(messages)))
                    time.sleep(.02)
                    continue
                now = rospy.Time.now().to_sec()
                stamps = [m.header.stamp.to_sec() for m in messages.values()]
                if any(not 0 <= now - stamp < .20 for stamp in stamps) or max(stamps) - min(stamps) > .08:
                    error = 'Stale or poorly aligned source timestamps'
                    time.sleep(.02)
                    continue
                tel, joint = messages['telemetry'], messages['joints']
                if tel.header.stamp.to_sec() == last_stamp:
                    time.sleep(.02)
                    continue
                if tel.mode != 'HOLDING' or not tel.ft_bias_ready or tel.fault_latched or tel.reason:
                    error = 'Need healthy HOLDING and completed bias calibration'
                    break
                try:
                    order = [joint.name.index(n) for n in NAMES]
                    q = [joint.position[i] for i in order]
                    v = [joint.velocity[i] for i in order]
                except (ValueError, IndexError):
                    error = 'Missing native joint positions/velocities'
                    break
                if max(abs(x) for x in v) > .01:
                    error = 'Robot moving; retry after settling'
                    break
                row = dict(label=label, time=now, raw_frame=messages['raw'].header.frame_id,
                           wrist_frame=messages['wrist'].header.frame_id, pose_frame=tel.header.frame_id)
                for key, message in messages.items():
                    row[key + '_stamp'] = message.header.stamp.to_sec()
                for key, values in [('raw', wrench(messages['raw'])), ('wrist', wrench(messages['wrist'])),
                                    ('corrected', tel.wrench_bias_corrected), ('filtered', tel.wrench_filtered),
                                    ('q', q), ('v', v)]:
                    row.update({key + '_' + str(i): float(x) for i, x in enumerate(values)})
                orientation = tel.actual_pose.orientation
                row.update(zip(('qx', 'qy', 'qz', 'qw'),
                               (orientation.x, orientation.y, orientation.z, orientation.w)))
                if not all(np.isfinite(value) for value in row.values() if isinstance(value, (float, int))):
                    error = 'Nonfinite sample'
                    break
                rows.append(row)
                last_stamp = tel.header.stamp.to_sec()
                error = ''
                time.sleep(.02)
            accepted = not error and len(rows) >= args.seconds * 20
            stats = summarize(rows) if rows else {}
            if rows and max(stats['joint_range_rad']) > .003:
                accepted = False
                error = 'Joint posture drifted during capture'
            if not accepted and not error:
                error = 'Insufficient samples'
            index = len(poses) + 1
            if rows:
                with (out / ('pose_%02d.csv' % index)).open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
            poses.append(dict(label=label, accepted=accepted, reason=error, samples=len(rows), statistics=stats))
            (out / 'summary.json').write_text(json.dumps(poses, indent=2))
            plot(out, poses)
            print(('SAVED' if accepted else 'REJECTED') + ': ' + label + ' ' + error, flush=True)
            if rows:
                print('  raw mean [N,N,N,Nm,Nm,Nm]: ' + str(np.round(stats['raw']['mean'], 3)), flush=True)
                print('  corrected mean: ' + str(np.round(stats['corrected']['mean'], 3)), flush=True)
    except (EOFError, KeyboardInterrupt):
        pass
    print('Saved: ' + str(out), flush=True)


if __name__ == '__main__':
    main()

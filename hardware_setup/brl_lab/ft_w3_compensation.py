#!/usr/bin/env python3
"""BRL_lab: subtract tool gravity + wrist_3-dependent F/T offset (ft_w3_model) from the wrist wrench.

Publishes <output> with the input frame/stamp. Withholds output without a fresh wrench,
an exact-stamp wrist orientation, or a joint state within 50 ms (HQP then reports stale F/T).
"""
import argparse
from collections import deque
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import ft_gravity  # noqa: E402  (rotation helper)
import ft_w3_model  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--input', default='/hqp_moveit/wrench')
    parser.add_argument('--output', default='/hqp_moveit/wrench_compensated')
    parser.add_argument('--joint-states', default='/joint_states')
    args = parser.parse_args([a for a in sys.argv[1:] if ':=' not in a])
    model = ft_w3_model.load(Path(args.model).expanduser())
    rospy.init_node('ft_w3_compensation')
    listener = tf.TransformListener()
    lock = threading.Lock()
    wrenches, joints = deque(maxlen=100), deque(maxlen=200)
    publisher = rospy.Publisher(args.output, WrenchStamped, queue_size=5)
    rospy.Subscriber(args.input, WrenchStamped, lambda m: (lock.acquire(), wrenches.append(m), lock.release()),
                     queue_size=20, tcp_nodelay=True)
    rospy.Subscriber(args.joint_states, JointState, lambda m: (lock.acquire(), joints.append(m), lock.release()),
                     queue_size=50, tcp_nodelay=True)
    joint_name = model.get('wrist_3_joint', 'wrist_3_joint')
    gravity_base = np.asarray(model['gravity_base'], dtype=float)
    rospy.loginfo('F/T w3 compensation %s -> %s (mass %.3f kg, K=%d)', args.input, args.output,
                  model['effective_mass_kg'], model['order'])
    last = 0.0
    while not rospy.is_shutdown():
        with lock:
            message = wrenches[-1] if wrenches else None
            states = list(joints)
        if message is None or message.header.stamp.to_sec() <= last:
            time.sleep(0.002)
            continue
        stamp = message.header.stamp.to_sec()
        if not 0 <= rospy.Time.now().to_sec() - stamp <= 0.2 or message.header.frame_id.lstrip('/') != model['wrench_frame']:
            time.sleep(0.002)
            continue
        state = min(states, key=lambda s: abs(s.header.stamp.to_sec() - stamp)) if states else None
        if state is None or abs(state.header.stamp.to_sec() - stamp) > 0.05 or joint_name not in state.name:
            rospy.logwarn_throttle(3, 'F/T w3 compensation: no joint state near the wrench stamp; withholding')
            time.sleep(0.002)
            continue
        try:
            _, quaternion = listener.lookupTransform(model['base_frame'], model['wrench_frame'], message.header.stamp)
        except tf.Exception:
            time.sleep(0.002)
            continue
        gravity = ft_gravity.rotation(quaternion).T @ gravity_base
        q3 = state.position[state.name.index(joint_name)]
        w = message.wrench
        value = np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]) - \
            ft_w3_model.predict(model, gravity, q3)
        out = WrenchStamped()
        out.header = message.header
        out.wrench.force.x, out.wrench.force.y, out.wrench.force.z = value[:3]
        out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z = value[3:]
        publisher.publish(out)
        last = stamp


if __name__ == '__main__':
    main()

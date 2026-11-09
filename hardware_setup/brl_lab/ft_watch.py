#!/usr/bin/env python3
"""BRL_lab: live check of the UR payload compensation (read-only).

Prints the UR /wrench (already payload-compensated by the UR controller) at 2 Hz:
|F|, |T| and the force expressed in the robot base frame. With a correct payload/CoG
the values stay near zero (|F| < ~2 N, |T| < ~0.15 Nm) in every static pose, with
nothing touching the tool. A mostly vertical force that persists in all poses means
the active payload mass differs from the mounted tool (about |F|/9.81 kg).
"""
import numpy as np
import rospy
import tf
from geometry_msgs.msg import WrenchStamped

latest = []


def receive(message):
    latest.append(message)
    del latest[:-50]


def main():
    rospy.init_node('brl_lab_ft_watch', anonymous=True)
    topic = rospy.get_param('~topic', '/wrench')
    base = rospy.get_param('~base_frame', 'base')
    listener = tf.TransformListener()
    rospy.Subscriber(topic, WrenchStamped, receive, queue_size=50)
    print('Watching %s (Ctrl+C to stop). Stop the robot and keep the tool untouched.' % topic)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()
        if not latest:
            continue
        samples = list(latest)
        frame = samples[-1].header.frame_id
        force = np.mean([[m.wrench.force.x, m.wrench.force.y, m.wrench.force.z] for m in samples], axis=0)
        torque = np.mean([[m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z] for m in samples], axis=0)
        try:
            _, quaternion = listener.lookupTransform(base, frame, rospy.Time(0))
            in_base = tf.transformations.quaternion_matrix(quaternion)[:3, :3].dot(force)
            base_text = 'F_base=(%+5.1f %+5.1f %+5.1f) N' % tuple(in_base)
        except tf.Exception:
            base_text = 'F_base=(no TF %s->%s)' % (base, frame)
        magnitude = np.linalg.norm(force)
        verdict = 'OK' if magnitude < 2.0 and np.linalg.norm(torque) < 0.15 else 'CHECK PAYLOAD'
        print('|F|=%5.2f N  |T|=%5.3f Nm  %s  -> %s' % (magnitude, np.linalg.norm(torque), base_text, verdict))


if __name__ == '__main__':
    main()

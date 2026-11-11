#!/usr/bin/env python
"""Kinematic stand-in for the field UR driver, for selftest.sh only (never on a real robot master).

Provides what the field launch files expect from ur_robot_driver, in this node's namespace:
  joint_states, wrench (frame ur5e_tool0_controller), speed_scaling_factor,
  ur_hardware_interface/robot_program_running, controller_manager/list_controllers,
  ur5e_scaled_pos_joint_traj_controller/{command, joints}.
Commands are executed like a joint trajectory controller (linear interpolation in time, exact
tracking). /selftest/push (WrenchStamped) adds a force to the measured wrench while it is sent."""
from __future__ import division, print_function
import threading
import rospy
from controller_manager_msgs.msg import ControllerState, HardwareInterfaceResources
from controller_manager_msgs.srv import ListControllers, ListControllersResponse
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory

NAMES = ['ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
         'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint']
CONTROLLER = 'ur5e_scaled_pos_joint_traj_controller'


class FakeDriver(object):
    def __init__(self):
        self.q = list(rospy.get_param('~initial', [0, -1.57, 1.57, -1.57, -1.57, 0]))
        self.lock = threading.Lock()
        self.segment = None  # (start time, [(t, positions)])
        self.push = [0.0] * 6
        self.push_until = rospy.Time(0)
        rospy.set_param('%s/joints' % CONTROLLER, NAMES)
        self.state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.wrench_pub = rospy.Publisher('wrench', WrenchStamped, queue_size=1)
        self.scaling_pub = rospy.Publisher('speed_scaling_factor', Float64, queue_size=1)
        self.program_pub = rospy.Publisher('ur_hardware_interface/robot_program_running', Bool, queue_size=1, latch=True)
        rospy.Subscriber('%s/command' % CONTROLLER, JointTrajectory, self.command, queue_size=1)
        rospy.Subscriber('/selftest/push', WrenchStamped, self.on_push, queue_size=1)
        rospy.Service('controller_manager/list_controllers', ListControllers, self.list_controllers)
        self.program_pub.publish(Bool(True))

    def list_controllers(self, _):
        c = ControllerState(name=CONTROLLER, state='running', type='position_controllers/ScaledJointTrajectoryController')
        c.claimed_resources = [HardwareInterfaceResources(
            hardware_interface='hardware_interface::PositionJointInterface', resources=NAMES)]
        return ListControllersResponse(controller=[c])

    def command(self, msg):
        if not msg.points or set(msg.joint_names) != set(NAMES):
            rospy.logwarn_throttle(2, 'fake driver: ignored command with unexpected joints')
            return
        order = [msg.joint_names.index(n) for n in NAMES]
        start = rospy.Time.now() if msg.header.stamp.is_zero() else msg.header.stamp
        points = [(p.time_from_start.to_sec(), [p.positions[i] for i in order]) for p in msg.points]
        with self.lock:
            self.segment = (start, [(0.0, list(self.q))] + points if points[0][0] > 0 else points)

    def on_push(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        with self.lock:
            self.push = [f.x, f.y, f.z, t.x, t.y, t.z]
            self.push_until = rospy.Time.now() + rospy.Duration(0.15)

    def step(self, now):
        with self.lock:
            if self.segment is None:
                return
            start, points = self.segment
            t = (now - start).to_sec()
            if t <= points[0][0]:
                return
            for (t0, q0), (t1, q1) in zip(points, points[1:]):
                if t <= t1:
                    a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                    self.q = [x + a * (y - x) for x, y in zip(q0, q1)]
                    return
            self.q = list(points[-1][1])

    def run(self):
        rate = rospy.Rate(250)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.step(now)
            s = JointState()
            s.header.stamp = now
            s.name = NAMES
            with self.lock:
                s.position = list(self.q)
                force = list(self.push) if now < self.push_until else [0.0] * 6
            s.velocity = [0.0] * 6
            s.effort = [0.0] * 6
            self.state_pub.publish(s)
            w = WrenchStamped()
            w.header.stamp = now
            w.header.frame_id = 'ur5e_tool0_controller'
            w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = force[:3]
            w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = force[3:]
            self.wrench_pub.publish(w)
            self.scaling_pub.publish(Float64(1.0))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('fake_ur_driver')
    FakeDriver().run()

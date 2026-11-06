import importlib.util
from pathlib import Path
import unittest

import rospy
from rci_action_manager.msg import MoveitOptimalStatus

spec = importlib.util.spec_from_file_location(
    'plan_forwarding', Path(__file__).resolve().parents[1]/'send_rviz_moveit_plan.py')
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class ForwardingTests(unittest.TestCase):
    def status(self):
        status = MoveitOptimalStatus()
        status.header.stamp = rospy.Time.from_sec(99.99)
        status.state = 'HOLDING'
        status.last_result = 'ABORTED'
        status.reason = 'goal tolerance timeout'
        status.command_enabled = status.ft_bias_ready = True
        return status

    def test_fresh_controller_can_receive_new_plan_after_goal_timeout(self):
        self.assertTrue(bridge.forwarding_ready(self.status(), 100., True))
        self.assertTrue(bridge.forwarding_ready(self.status(), 100., False))

    def test_retained_timeout_does_not_block_tracking_replacement(self):
        status = self.status(); status.state = 'TRACKING'
        self.assertTrue(bridge.forwarding_ready(status, 100., True))
        self.assertFalse(bridge.forwarding_ready(status, 100., False))

    def test_current_faults_and_input_timeouts_remain_blocked(self):
        for reason in ['latched fault: command-to-measured joint gap exceeded',
                       'HQP infeasible or numerical failure', 'measured joint limit exceeded',
                       'External Control inactive, scaling stale or speed zero',
                       'F/T input unavailable or stale', 'joint state unavailable or stale']:
            with self.subTest(reason=reason):
                status = self.status(); status.reason = reason
                self.assertFalse(bridge.forwarding_ready(status, 100., True))

    def test_bias_and_command_gates_remain_required(self):
        for name in ['ft_bias_ready', 'command_enabled']:
            status = self.status(); setattr(status, name, False)
            self.assertFalse(bridge.forwarding_ready(status, 100., True))

    def test_stale_and_future_status_remain_blocked(self):
        for stamp in [99., 100.1]:
            status = self.status(); status.header.stamp = rospy.Time.from_sec(stamp)
            self.assertFalse(bridge.forwarding_ready(status, 100., True))

    def test_other_abort_reasons_are_not_silently_cleared(self):
        status = self.status(); status.reason = 'path tolerance exceeded'
        self.assertFalse(bridge.forwarding_ready(status, 100., True))


if __name__ == '__main__':
    unittest.main()

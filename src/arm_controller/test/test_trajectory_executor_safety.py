import threading
import unittest
from types import SimpleNamespace

from rclpy.action import CancelResponse

from arm_controller.trajectory_executor import TrajectoryExecutor
from arm_controller.trajectory_safety import TrajectoryValidationError


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def error(self, _message):
        pass

    def warning(self, _message):
        pass


class TrajectoryExecutorSafetyTests(unittest.TestCase):
    def test_cancel_immediately_latches_and_publishes_estop(self):
        executor = object.__new__(TrajectoryExecutor)
        executor.command_lock = threading.RLock()
        executor.emergency_stop_active = False
        executor.estop_reset_requested = False
        executor.should_cancel = False
        executor.estop_pub = _Publisher()
        executor.get_logger = lambda: _Logger()

        response = executor.cancel_callback(object())

        self.assertEqual(response, CancelResponse.ACCEPT)
        self.assertTrue(executor.emergency_stop_active)
        self.assertTrue(executor.should_cancel)
        self.assertEqual(len(executor.estop_pub.messages), 1)
        self.assertTrue(executor.estop_pub.messages[0].data)

    def test_component_tolerances_are_rejected(self):
        executor = object.__new__(TrajectoryExecutor)
        request = SimpleNamespace(
            trajectory=SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=0, nanosec=0)
                )
            ),
            path_tolerance=[],
            goal_tolerance=[],
            component_path_tolerance=[object()],
            component_goal_tolerance=[],
            multi_dof_trajectory=SimpleNamespace(joint_names=[], points=[]),
            goal_time_tolerance=SimpleNamespace(sec=0, nanosec=0),
        )

        with self.assertRaisesRegex(
            TrajectoryValidationError,
            "per-component",
        ):
            executor._validate_goal_contract(request)


if __name__ == "__main__":
    unittest.main()

import unittest

from arm_controller.projects.caps.cap_sequence_simulation.cap_sequence_simulation_safety import (
    ACTION_STATUS_SUCCEEDED,
    DEFAULT_CAP_IDS,
    FOLLOW_JOINT_TRAJECTORY_SUCCESS,
    MOVEIT_SUCCESS,
    CapSequenceSafetyError,
    CapSequenceState,
    OfflineBackendGraph,
    StageChecks,
    TransferStage,
    require_action_success,
    require_cartesian_path_fraction,
    require_motion_plan_success,
    require_offline_backend_graph,
)


class OfflineBackendGraphTests(unittest.TestCase):
    def _valid_graph(self, **overrides):
        values = {
            "node_names": (
                "/move_group",
                "/trajectory_executor",
                "/servo_node",
            ),
            "joint_command_publishers": ("/trajectory_executor",),
            "joint_command_subscribers": ("/servo_node",),
            "joint_state_publishers": ("/servo_node",),
            "raspberry_status_publishers": (),
        }
        values.update(overrides)
        return OfflineBackendGraph(**values)

    def test_accepts_exact_executor_to_simulator_graph(self):
        require_offline_backend_graph(self._valid_graph())

    def test_rejects_udp_bridge_in_nodes_or_topic_authorities(self):
        for override in (
            {"node_names": ("/servo_node", "/udp_servo_bridge")},
            {
                "joint_command_subscribers": (
                    "/servo_node",
                    "/hardware/udp_servo_bridge",
                )
            },
            {"joint_state_publishers": ("/udp_servo_bridge",)},
        ):
            with self.subTest(override=override):
                with self.assertRaisesRegex(
                    CapSequenceSafetyError, "udp_servo_bridge"
                ):
                    require_offline_backend_graph(
                        self._valid_graph(**override)
                    )

    def test_rejects_any_raspberry_status_authority(self):
        graph = self._valid_graph(
            raspberry_status_publishers=("/renamed_hardware_bridge",)
        )
        with self.assertRaisesRegex(
            CapSequenceSafetyError, "raspberry/status"
        ):
            require_offline_backend_graph(graph)

    def test_rejects_gui_or_other_extra_command_authority(self):
        graph = self._valid_graph(
            joint_command_publishers=(
                "/trajectory_executor",
                "/arm_control_gui",
            )
        )
        with self.assertRaisesRegex(CapSequenceSafetyError, "exactly"):
            require_offline_backend_graph(graph)


class ResultValidationTests(unittest.TestCase):
    def test_requires_nearly_complete_cartesian_fraction(self):
        require_cartesian_path_fraction(0.999)
        require_cartesian_path_fraction(1.0)
        for fraction in (0.998999, float("nan"), float("inf"), 1.001, True):
            with self.subTest(fraction=fraction):
                with self.assertRaises(CapSequenceSafetyError):
                    require_cartesian_path_fraction(fraction)

    def test_validates_moveit_motion_plan_code(self):
        require_motion_plan_success(MOVEIT_SUCCESS)
        for code in (0, -1, True, None):
            with self.subTest(code=code):
                with self.assertRaises(CapSequenceSafetyError):
                    require_motion_plan_success(code)

    def test_action_requires_successful_envelope_and_domain_result(self):
        require_action_success(
            ACTION_STATUS_SUCCEEDED,
            MOVEIT_SUCCESS,
            expected_result_error_code=MOVEIT_SUCCESS,
        )
        require_action_success(
            ACTION_STATUS_SUCCEEDED,
            FOLLOW_JOINT_TRAJECTORY_SUCCESS,
            expected_result_error_code=FOLLOW_JOINT_TRAJECTORY_SUCCESS,
        )
        for status, result, expected in (
            (5, MOVEIT_SUCCESS, MOVEIT_SUCCESS),
            (ACTION_STATUS_SUCCEEDED, -1, MOVEIT_SUCCESS),
            (True, MOVEIT_SUCCESS, MOVEIT_SUCCESS),
        ):
            with self.subTest(status=status, result=result):
                with self.assertRaises(CapSequenceSafetyError):
                    require_action_success(
                        status,
                        result,
                        expected_result_error_code=expected,
                    )


class CapSequenceStateTests(unittest.TestCase):
    _TRANSFER = (
        TransferStage.PREGRASP_REACHED,
        TransferStage.GRASP_REACHED,
        TransferStage.GRIPPER_CLOSED,
        TransferStage.ATTACHED,
        TransferStage.LIFTED,
        TransferStage.PLACE_ABOVE_REACHED,
        TransferStage.PLACE_REACHED,
        TransferStage.GRIPPER_OPENED,
        TransferStage.DETACHED,
        TransferStage.RETREATED,
        TransferStage.COMPLETE,
    )

    def test_happy_path_enforces_all_seven_transfers_and_attachment(self):
        state = CapSequenceState()
        for cap_id in DEFAULT_CAP_IDS:
            for stage in self._TRANSFER:
                state.advance(cap_id, stage)
                if stage is TransferStage.ATTACHED:
                    self.assertEqual(state.attached_cap_id, cap_id)
                if stage is TransferStage.DETACHED:
                    self.assertIsNone(state.attached_cap_id)

        self.assertTrue(state.is_complete)
        self.assertEqual(state.completed_cap_ids, DEFAULT_CAP_IDS)
        state.assert_complete()

    def test_requires_exactly_seven_unique_caps(self):
        for cap_ids in (
            DEFAULT_CAP_IDS[:6],
            (*DEFAULT_CAP_IDS, "cap_7"),
            (*DEFAULT_CAP_IDS[:6], "cap_0"),
        ):
            with self.subTest(cap_ids=cap_ids):
                with self.assertRaises(CapSequenceSafetyError):
                    CapSequenceState(cap_ids)

    def test_attach_before_close_fails_and_latches_sequence(self):
        state = CapSequenceState()
        cap_id = state.active_cap_id
        state.advance(cap_id, TransferStage.PREGRASP_REACHED)
        state.advance(cap_id, TransferStage.GRASP_REACHED)

        with self.assertRaisesRegex(CapSequenceSafetyError, "expected"):
            state.advance(cap_id, TransferStage.ATTACHED)
        self.assertIsNotNone(state.failure_reason)
        with self.assertRaisesRegex(
            CapSequenceSafetyError, "sequence is failed"
        ):
            state.advance(cap_id, TransferStage.GRIPPER_CLOSED)

    def test_detach_before_open_fails_and_preserves_attached_cap(self):
        state = CapSequenceState()
        cap_id = state.active_cap_id
        for stage in self._TRANSFER[:7]:
            state.advance(cap_id, stage)
        self.assertEqual(state.stage_for(cap_id), TransferStage.PLACE_REACHED)
        self.assertEqual(state.attached_cap_id, cap_id)

        with self.assertRaisesRegex(CapSequenceSafetyError, "expected"):
            state.advance(cap_id, TransferStage.DETACHED)
        self.assertEqual(state.attached_cap_id, cap_id)

    def test_next_cap_cannot_start_before_current_cap_completes(self):
        state = CapSequenceState()
        with self.assertRaisesRegex(
            CapSequenceSafetyError, "expected active cap"
        ):
            state.advance(DEFAULT_CAP_IDS[1], TransferStage.PREGRASP_REACHED)
        self.assertEqual(state.completed_cap_ids, ())

    def test_failed_stage_evidence_prevents_transition_and_future_motion(self):
        state = CapSequenceState()
        cap_id = state.active_cap_id
        checks = StageChecks(
            motion_plan_error_code=-1,
            action_status=ACTION_STATUS_SUCCEEDED,
            action_result_error_code=MOVEIT_SUCCESS,
            expected_action_result_error_code=MOVEIT_SUCCESS,
        )

        with self.assertRaisesRegex(CapSequenceSafetyError, "motion-plan"):
            state.advance_checked(
                cap_id,
                TransferStage.PREGRASP_REACHED,
                checks,
            )
        self.assertEqual(state.stage_for(cap_id), TransferStage.QUEUED)
        self.assertIsNotNone(state.failure_reason)
        with self.assertRaisesRegex(
            CapSequenceSafetyError, "sequence is failed"
        ):
            state.advance(cap_id, TransferStage.PREGRASP_REACHED)

    def test_checked_transition_accepts_complete_cartesian_evidence(self):
        state = CapSequenceState()
        cap_id = state.active_cap_id
        checks = StageChecks(
            motion_plan_error_code=MOVEIT_SUCCESS,
            cartesian_fraction=0.999,
            action_status=ACTION_STATUS_SUCCEEDED,
            action_result_error_code=MOVEIT_SUCCESS,
            expected_action_result_error_code=MOVEIT_SUCCESS,
        )
        state.advance_checked(
            cap_id,
            TransferStage.PREGRASP_REACHED,
            checks,
        )
        self.assertEqual(
            state.stage_for(cap_id), TransferStage.PREGRASP_REACHED
        )

    def test_completion_invariant_rejects_partial_sequence(self):
        state = CapSequenceState()
        for cap_id in DEFAULT_CAP_IDS[:6]:
            for stage in self._TRANSFER:
                state.advance(cap_id, stage)
        with self.assertRaisesRegex(
            CapSequenceSafetyError, "completion invariant"
        ):
            state.assert_complete()


if __name__ == "__main__":
    unittest.main()

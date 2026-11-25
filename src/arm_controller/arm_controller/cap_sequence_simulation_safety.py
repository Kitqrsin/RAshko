"""ROS-free safety contracts for the offline seven-cap sequence.

The future simulation node can translate ROS graph endpoints and action/service
results into these small value objects.  Keeping this module free of ``rclpy``
allows every fail-closed rule to be unit tested without a running ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence


MOVEIT_SUCCESS = 1
FOLLOW_JOINT_TRAJECTORY_SUCCESS = 0
ACTION_STATUS_SUCCEEDED = 4
MIN_CARTESIAN_FRACTION = 0.999
DEFAULT_CAP_IDS = tuple(f"cap_{index}" for index in range(7))


class CapSequenceSafetyError(RuntimeError):
    """Raised when an offline sequence safety invariant is not satisfied."""


@dataclass(frozen=True)
class OfflineBackendGraph:
    """Relevant ROS graph endpoint authorities captured by a caller.

    Node names may be fully qualified.  Endpoint lists must contain the node
    name reported by ROS for each publisher or subscriber on that topic.
    """

    node_names: tuple[str, ...] = ()
    joint_command_publishers: tuple[str, ...] = ()
    joint_command_subscribers: tuple[str, ...] = ()
    joint_state_publishers: tuple[str, ...] = ()
    raspberry_status_publishers: tuple[str, ...] = ()


def require_offline_backend_graph(graph: OfflineBackendGraph) -> None:
    """Require an executor-to-simulator graph with no hardware authority.

    This checks DDS authorities only.  A launch must additionally isolate the
    simulation from the Raspberry Pi at the process/network/credential layer.
    """

    if not isinstance(graph, OfflineBackendGraph):
        raise CapSequenceSafetyError(
            "offline backend graph has an invalid type"
        )

    endpoint_groups = (
        graph.node_names,
        graph.joint_command_publishers,
        graph.joint_command_subscribers,
        graph.joint_state_publishers,
        graph.raspberry_status_publishers,
    )
    normalized_groups = tuple(
        tuple(_node_basename(name) for name in group)
        for group in endpoint_groups
    )
    all_nodes = {
        name for group in normalized_groups for name in group
    }
    if "udp_servo_bridge" in all_nodes:
        raise CapSequenceSafetyError(
            "udp_servo_bridge is present in the offline ROS graph"
        )
    if graph.raspberry_status_publishers:
        raise CapSequenceSafetyError(
            "/raspberry/status has a publisher in offline simulation"
        )

    _require_single_authority(
        normalized_groups[1],
        "trajectory_executor",
        "/joint_commands publisher",
    )
    _require_single_authority(
        normalized_groups[2],
        "servo_node",
        "/joint_commands subscriber",
    )
    _require_single_authority(
        normalized_groups[3],
        "servo_node",
        "/joint_states publisher",
    )


def _node_basename(value: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise CapSequenceSafetyError("ROS graph contains an invalid node name")
    return value.strip().rstrip("/").rsplit("/", 1)[-1]


def _require_single_authority(
    actual: tuple[str, ...],
    expected: str,
    description: str,
) -> None:
    if actual != (expected,):
        raise CapSequenceSafetyError(
            f"offline {description} must be exactly {expected!r}; "
            f"got {actual!r}"
        )


def require_cartesian_path_fraction(fraction: float) -> None:
    """Require a complete, finite MoveIt Cartesian path."""

    if isinstance(fraction, bool):
        raise CapSequenceSafetyError("Cartesian path fraction is invalid")
    try:
        value = float(fraction)
    except (TypeError, ValueError) as exc:
        raise CapSequenceSafetyError(
            "Cartesian path fraction is invalid"
        ) from exc
    if (
        not math.isfinite(value)
        or value < MIN_CARTESIAN_FRACTION
        or value > 1.0
    ):
        raise CapSequenceSafetyError(
            "Cartesian path fraction must be finite and within "
            f"{MIN_CARTESIAN_FRACTION:.3f}..1.000; got {value!r}"
        )


def require_motion_plan_success(error_code: int) -> None:
    """Validate ``moveit_msgs/MoveItErrorCodes`` from a planning result."""

    _require_exact_code(error_code, MOVEIT_SUCCESS, "motion-plan error code")


def require_action_success(
    status: int,
    result_error_code: int,
    *,
    expected_result_error_code: int,
) -> None:
    """Validate both the ROS action envelope and its domain result code."""

    _require_exact_code(status, ACTION_STATUS_SUCCEEDED, "action status")
    _require_exact_code(
        result_error_code,
        expected_result_error_code,
        "action result error code",
    )


def _require_exact_code(actual: int, expected: int, description: str) -> None:
    if (
        type(actual) is not int
        or type(expected) is not int
        or actual != expected
    ):
        raise CapSequenceSafetyError(
            f"{description} must be {expected}; got {actual!r}"
        )


@dataclass(frozen=True)
class StageChecks:
    """Optional planning/execution evidence for one state transition."""

    motion_plan_error_code: int | None = None
    cartesian_fraction: float | None = None
    action_status: int | None = None
    action_result_error_code: int | None = None
    expected_action_result_error_code: int | None = None

    def validate(self) -> None:
        checked = False
        if self.motion_plan_error_code is not None:
            require_motion_plan_success(self.motion_plan_error_code)
            checked = True
        if self.cartesian_fraction is not None:
            require_cartesian_path_fraction(self.cartesian_fraction)
            checked = True

        action_values = (
            self.action_status,
            self.action_result_error_code,
            self.expected_action_result_error_code,
        )
        if any(value is not None for value in action_values):
            if any(value is None for value in action_values):
                raise CapSequenceSafetyError(
                    "action status, result code, and expected result code "
                    "must be supplied together"
                )
            require_action_success(
                self.action_status,
                self.action_result_error_code,
                expected_result_error_code=(
                    self.expected_action_result_error_code
                ),
            )
            checked = True
        if not checked:
            raise CapSequenceSafetyError(
                "stage has no planning or action evidence"
            )


class TransferStage(str, Enum):
    QUEUED = "queued"
    PREGRASP_REACHED = "pregrasp_reached"
    GRASP_REACHED = "grasp_reached"
    GRIPPER_CLOSED = "gripper_closed"
    ATTACHED = "attached"
    LIFTED = "lifted"
    PLACE_ABOVE_REACHED = "place_above_reached"
    PLACE_REACHED = "place_reached"
    GRIPPER_OPENED = "gripper_opened"
    DETACHED = "detached"
    RETREATED = "retreated"
    COMPLETE = "complete"


_NEXT_STAGE = {
    TransferStage.QUEUED: TransferStage.PREGRASP_REACHED,
    TransferStage.PREGRASP_REACHED: TransferStage.GRASP_REACHED,
    TransferStage.GRASP_REACHED: TransferStage.GRIPPER_CLOSED,
    TransferStage.GRIPPER_CLOSED: TransferStage.ATTACHED,
    TransferStage.ATTACHED: TransferStage.LIFTED,
    TransferStage.LIFTED: TransferStage.PLACE_ABOVE_REACHED,
    TransferStage.PLACE_ABOVE_REACHED: TransferStage.PLACE_REACHED,
    TransferStage.PLACE_REACHED: TransferStage.GRIPPER_OPENED,
    TransferStage.GRIPPER_OPENED: TransferStage.DETACHED,
    TransferStage.DETACHED: TransferStage.RETREATED,
    TransferStage.RETREATED: TransferStage.COMPLETE,
}


class CapSequenceState:
    """Fail-fast ordering model for exactly seven sequential cap transfers."""

    def __init__(self, cap_ids: Sequence[str] = DEFAULT_CAP_IDS) -> None:
        if isinstance(cap_ids, (str, bytes)):
            raise CapSequenceSafetyError("cap IDs must be a sequence")
        normalized = tuple(cap_ids)
        if (
            len(normalized) != 7
            or any(
                not isinstance(cap_id, str) or not cap_id
                for cap_id in normalized
            )
            or len(set(normalized)) != 7
        ):
            raise CapSequenceSafetyError(
                "offline sequence requires exactly seven unique non-empty "
                "cap IDs"
            )
        self.cap_ids = normalized
        self._stages = {
            cap_id: TransferStage.QUEUED for cap_id in self.cap_ids
        }
        self._active_index = 0
        self._completed: list[str] = []
        self.attached_cap_id: str | None = None
        self.failure_reason: str | None = None

    @property
    def active_cap_id(self) -> str | None:
        if self._active_index >= len(self.cap_ids):
            return None
        return self.cap_ids[self._active_index]

    @property
    def completed_cap_ids(self) -> tuple[str, ...]:
        return tuple(self._completed)

    @property
    def is_complete(self) -> bool:
        return (
            self.failure_reason is None
            and self.active_cap_id is None
            and self.attached_cap_id is None
            and self.completed_cap_ids == self.cap_ids
            and all(
                stage is TransferStage.COMPLETE
                for stage in self._stages.values()
            )
        )

    def stage_for(self, cap_id: str) -> TransferStage:
        try:
            return self._stages[cap_id]
        except KeyError as exc:
            raise CapSequenceSafetyError(f"unknown cap ID {cap_id!r}") from exc

    def advance_checked(
        self,
        cap_id: str,
        next_stage: TransferStage,
        checks: StageChecks,
    ) -> None:
        """Validate ROS-derived evidence, then atomically advance or fail."""

        try:
            if not isinstance(checks, StageChecks):
                raise CapSequenceSafetyError(
                    "stage checks have an invalid type"
                )
            checks.validate()
        except CapSequenceSafetyError as exc:
            self.abort(f"{cap_id} {next_stage.value}: {exc}")
            raise CapSequenceSafetyError(self.failure_reason) from exc
        self.advance(cap_id, next_stage)

    def advance(self, cap_id: str, next_stage: TransferStage) -> None:
        """Apply one valid transition, latching any ordering failure."""

        if self.failure_reason is not None:
            raise CapSequenceSafetyError(
                f"sequence is failed: {self.failure_reason}"
            )
        if not isinstance(next_stage, TransferStage):
            self._fail(f"invalid transfer stage {next_stage!r}")
        if self.active_cap_id is None:
            self._fail("all seven cap transfers are already complete")
        if cap_id != self.active_cap_id:
            self._fail(
                f"expected active cap {self.active_cap_id!r}, got {cap_id!r}"
            )

        current = self.stage_for(cap_id)
        expected = _NEXT_STAGE.get(current)
        if next_stage is not expected:
            expected_text = expected.value if expected is not None else "none"
            self._fail(
                f"invalid {cap_id} transition {current.value} -> "
                f"{next_stage.value}; expected {expected_text}"
            )

        if next_stage is TransferStage.ATTACHED:
            if self.attached_cap_id is not None:
                self._fail(
                    f"cannot attach {cap_id}; "
                    f"{self.attached_cap_id} is attached"
                )
            self.attached_cap_id = cap_id
        elif next_stage is TransferStage.DETACHED:
            if self.attached_cap_id != cap_id:
                self._fail(
                    f"cannot detach {cap_id}; attached cap is "
                    f"{self.attached_cap_id!r}"
                )
            self.attached_cap_id = None
        elif (
            current in {
                TransferStage.ATTACHED,
                TransferStage.LIFTED,
                TransferStage.PLACE_ABOVE_REACHED,
                TransferStage.PLACE_REACHED,
                TransferStage.GRIPPER_OPENED,
            }
            and self.attached_cap_id != cap_id
        ):
            self._fail(f"{cap_id} must remain attached through placement")

        self._stages[cap_id] = next_stage
        if next_stage is TransferStage.COMPLETE:
            if self.attached_cap_id is not None:
                self._fail("a transfer cannot complete with a cap attached")
            self._completed.append(cap_id)
            self._active_index += 1

    def abort(self, reason: str) -> None:
        """Latch the first failure; later errors cannot erase its cause."""

        if not isinstance(reason, str) or not reason.strip():
            reason = "unspecified cap-sequence failure"
        if self.failure_reason is None:
            self.failure_reason = reason.strip()

    def assert_complete(self) -> None:
        """Enforce the final exactly-seven, none-attached invariant."""

        if not self.is_complete:
            raise CapSequenceSafetyError(
                "seven-cap completion invariant is not satisfied: "
                f"completed={self.completed_cap_ids!r}, "
                f"active={self.active_cap_id!r}, "
                f"attached={self.attached_cap_id!r}, "
                f"failure={self.failure_reason!r}"
            )

    def _fail(self, reason: str) -> None:
        self.abort(reason)
        raise CapSequenceSafetyError(self.failure_reason)

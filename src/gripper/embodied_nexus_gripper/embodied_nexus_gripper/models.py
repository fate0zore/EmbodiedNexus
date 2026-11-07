"""Shared, model-neutral gripper feedback values."""

from dataclasses import dataclass


@dataclass(frozen=True)
class GripperFeedback:
    """Normalized feedback consumed by the common gripper controller."""

    moving: bool | None
    reached: bool | None
    stationary: bool | None
    contact_valid: bool | None = None
    contact_detected: bool | None = None
    contact_object_ids: tuple[str, ...] = ()

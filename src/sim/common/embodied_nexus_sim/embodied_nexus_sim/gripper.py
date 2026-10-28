"""Public ROS simulation backend for ms42dc."""

from .adapters.ms42dc.backend import Ros2SimGripperBackend

__all__ = ["Ros2SimGripperBackend"]

# Continuous joint-velocity control

The `joint_velocity` interface streams measured-state joint velocities under the existing arm control lease. It is intended for short, closed-loop updates from local trajectory tracking or visual servoing.

The first nonzero velocity command stops any prior motion and enters Aubo servo mode. Subsequent nonzero commands update the servo target without stopping between updates. A zero-velocity command stops the stream, exits servo mode and reports whether the arm is stationary; the lease and motion-enable state remain available for another path segment. Releasing the lease, lease expiry, the velocity watchdog, a command error, or an explicit stop requests the normal stop path.

Each `SetArmJointVelocity` response includes:

- `controller_accepted`: the vendor controller accepted this update.
- `retry_next_cycle`: the Aubo servo endpoint returned its busy result; the caller may retry on the next control cycle without treating the update as accepted.
- `commanded_positions_rad`: the joint target sent to the servo controller.
- `stationary_confirmed`: true after a zero command only when stop feedback confirms stationary joints.

The REST API and Python SDK preserve the same response fields. Device limits, lease checks, teach-mode exclusion, fresh feedback checks and watchdog behavior remain enforced by the arm controller. A caller must stop on rejection, expired feedback, lost lease, or missing stationary confirmation; it must not interpret a busy/retry response as an accepted command.

The arm state response also exposes `continuous_control_active`, `joint_velocity_command_age_sec`, `joint_velocity_command_stamp_sec`, and `feedback_timestamp_source` (`host_receive` for the time the arm node received the feedback). Use those with `stamp_sec`, `feedback_age_sec`, and `stationary_confirmed` to distinguish a live servo stream, an old command, stale joint feedback, and a confirmed stop. RGB-D consumers can require the camera and arm timestamps to use the same clock source before combining their measurements.

Joint move actions retain their configured completion tolerance and timeout. Timeout or cancellation requests stop and leaves the operation failed/cancelled unless the existing completion criteria were actually met. Increasing a timeout does not convert a failed action into success.

After changing the service definition, rebuild the ROS interface and arm node from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to embodied_nexus_arm
source install/setup.bash
```

Mock regression commands from this package and the SDK directory are `python -m pytest -q test` and `python -m pytest -q tests`. Hardware acceptance should separately verify a read-only state query, a small joint move and stop, repeated nonzero velocity updates without inter-update stops, explicit zero and stationary confirmation, lease/watchdog stop, and the action timeout path.

# C16 Hardware Acceptance (2026-10-18)

## Software acceptance

- Built `lslidar_msgs`, `embodied_nexus_lidar_interfaces`, `lslidar_driver`, and `embodied_nexus_lidar` on the Yahboom Ubuntu 22.04 / ROS 2 Humble host.
- All 17 package unit tests passed, including DIFOP validation, cloud layout and filtering, LaserScan boundaries, control leases, authenticated HTTP/WebSocket, and binary frame decoding.
- `validate_lidar_modes.py` passed for mock `ros2`, `web`, and `both`, plus `ros2_topics` forwarding. Authenticated WebSocket control and stream decoding passed.
- The existing camera and gripper nodes remained running throughout; the lidar driver released UDP ports 2368/2369 on exit.

## Real device observations

The C16 at `192.168.1.200` was identified by the bundled driver as `C16`. The wrapper received valid DIFOP packets with header `a5ff005a`, protocol revision 4, and a readable 10 Hz speed before the first stop request. At that point the wrapper reported `connected=true`, `data_fresh=true`, one point-cloud frame, `pointcloud_age_sec=0.648`, `pointcloud_fps=0.0` (only one frame was available for that sample), and zero dropped frames. The hardware motor state correctly remained `unknown`; the driver exposes no independent motor-running confirmation.

The inspection process then requested stop and logged `shutdown lidar stop command sent=True`. With the motor stopped, later DIFOP packets continued to arrive but no longer contained a speed value recognized as 5/10/20 Hz. The test used the previously recorded 10 Hz as its restore value and sent a 10 Hz setting command under a valid lease. UDP send succeeded, but DIFOP did not confirm 10 Hz within the readback window. The acceptance script therefore withheld motor start and all 5/10/20 Hz test changes. Lease release and the final stop request both succeeded; the reported hardware motor state remains unknown.

## Remaining hardware acceptance at the end of the 2026-10-18 run

Real acquisition was observed at startup, but a sustained frequency/bandwidth sample and the motor start/stop plus 5/10/20 Hz readback sequence remained unverified at the end of that run. The validator used then required the stopped-state baseline command to read back before motor start:

```bash
python3 src/lidar/embodied_nexus_lidar/test/validate_c16_hardware.py \
  --enable-motor-test --recorded-original-speed-hz 10
```

The `10` value is valid only because it was read live before the first stop request during this acceptance session. That validator withheld start when DIFOP omitted the stopped-state speed field. The product API reports command-send success separately from device confirmation for this reason. The updated procedure sends the baseline before start and confirms it from DIFOP after start, before changing test speeds; see [lidar-c16.md](lidar-c16.md).

## Follow-up on 2026-10-19

- **Acceptance-flow change: passed (software/procedure only).** The validator now sets the recorded 10 Hz baseline before start, requires running-state DIFOP readback before trying 5/10/20 Hz, and restores the baseline before release/stop. Script compilation, diff checks, package tests, and execution of the updated guard path passed. This does not change the C16 motor's hardware result below.
- The updated validator was run from `/tmp/EmbodiedNexus-lidar-validation-20261019` on the Jetson, leaving its Git-managed source tree unchanged. The C16 sent protocol-v4 DIFOP packets; the recorded baseline was the previously live-read 10 Hz.
- First attempt: stop, 10 Hz baseline, and motor-start commands were sent. DIFOP did not report a recognized speed within 4 seconds, and `point_sequence` remained 0, so 5/10/20 Hz tests were skipped.
- One follow-up attempt increased the baseline readback window to 10 seconds. Stop, 10 Hz baseline, and motor-start commands were sent successfully, but DIFOP still did not report a recognized speed; `motor_state` remained `unknown`, and `point_sequence` remained 0. No 5/10/20 Hz speed test was run.
- During the 10-second attempt, cleanup sent the 10 Hz restore command, which also had no confirmed speed readback. Lease release returned `stop_sent=true`, and the lidar node exited cleanly. The operator could not observe whether the motor physically rotated; the final hardware motor state and speed remain unconfirmed.

## Follow-up on 2026-10-23

- The C16 motor-start packet was updated to include the requested 10 Hz value when stopped-state DIFOP omits speed. The Jetson rebuilt the lidar packages and passed 18 package tests before this hardware attempt.
- With the robot confirmed ready, the managed lidar was stopped and the updated local validator was run against the real C16 using the previously live-read 10 Hz baseline. Stop, baseline-speed, and motor-start UDP sends all succeeded at the host API level.
- The device continued sending valid protocol-v4 DIFOP packets, but their speed field remained unrecognized (`speed_hz=null`). No point cloud arrived (`point_sequence=0`, `pointcloud_fps=0`); the 10 Hz readback timed out after 10 seconds, so the validator skipped all 5/10/20 Hz changes.
- Cleanup sent the 10 Hz restore command, released the lease with `stop_sent=true`, and shut down its standalone lidar node cleanly. The Core-managed lidar was then started again and is `ready`/connected with fresh DIFOP, but `data_fresh=false`, motor state/speed unknown, and no points.
- **Hardware acceptance remains failed/unconfirmed.** `sendto` success confirms only that the host submitted the UDP datagram; the C16 did not confirm speed and produced no point cloud. The packet fix does not explain this remaining hardware-side failure.

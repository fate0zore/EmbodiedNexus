# C16 Hardware Acceptance (2026-10-18)

## Software acceptance

- Built `lslidar_msgs`, `embodied_nexus_lidar_interfaces`, `lslidar_driver`, and `embodied_nexus_lidar` on the Yahboom Ubuntu 22.04 / ROS 2 Humble host.
- All 17 package unit tests passed, including DIFOP validation, cloud layout and filtering, LaserScan boundaries, control leases, authenticated HTTP/WebSocket, and binary frame decoding.
- `validate_lidar_modes.py` passed for mock `ros2`, `web`, and `both`, plus `ros2_topics` forwarding. Authenticated WebSocket control and stream decoding passed.
- The existing camera and gripper nodes remained running throughout; the lidar driver released UDP ports 2368/2369 on exit.

## Real device observations

The C16 at `192.168.1.200` was identified by the bundled driver as `C16`. The wrapper received valid DIFOP packets with header `a5ff005a`, protocol revision 4, and a readable 10 Hz speed before the first stop request. At that point the wrapper reported `connected=true`, `data_fresh=true`, one point-cloud frame, `pointcloud_age_sec=0.648`, `pointcloud_fps=0.0` (only one frame was available for that sample), and zero dropped frames. The hardware motor state correctly remained `unknown`; the driver exposes no independent motor-running confirmation.

The inspection process then requested stop and logged `shutdown lidar stop command sent=True`. With the motor stopped, later DIFOP packets continued to arrive but no longer contained a speed value recognized as 5/10/20 Hz. The test used the previously recorded 10 Hz as its restore value and sent a 10 Hz setting command under a valid lease. UDP send succeeded, but DIFOP did not confirm 10 Hz within the readback window. The acceptance script therefore withheld motor start and all 5/10/20 Hz test changes. Lease release and the final stop request both succeeded; the reported hardware motor state remains unknown.

## Remaining hardware acceptance

Real acquisition was observed at startup, but a sustained frequency/bandwidth sample and the motor start/stop plus 5/10/20 Hz readback sequence remain unverified. Repeat only after the C16 provides a readable speed after the baseline restore command. The script enforces that readback before motor start:

```bash
python3 src/lidar/embodied_nexus_lidar/test/validate_c16_hardware.py \
  --enable-motor-test --recorded-original-speed-hz 10
```

The `10` value is valid only because it was read live before the first stop request during this acceptance session. If DIFOP cannot confirm the restore value, do not bypass the script's guard or issue a motor-start command. The product API reports command-send success separately from device confirmation for this reason.

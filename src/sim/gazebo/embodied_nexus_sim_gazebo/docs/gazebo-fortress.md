# Gazebo Fortress local simulation

This package uses the ROS 2 Humble / Gazebo Fortress pairing and runs in WSL2 Ubuntu 22.04. Install the runtime and build dependencies there:

```bash
sudo apt-get install -y ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-ros2-controllers ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  libpcl-dev python3-colcon-common-extensions python3-opencv \
  python3-starlette python3-uvicorn python3-websockets python3-pip
```

Source `/opt/ros/humble/setup.bash` and the workspace `install/setup.bash` in every ROS terminal. `libpcl-dev` is needed by the existing LSLidar driver when building the full dependency chain. Install the Python SDK in the Windows Python environment with `python -m pip install -e sdk/python`.

The Ubuntu 22.04 ROS apt Python packages include an older WebSocket stack that cannot serve the SDK's streaming endpoints under Python 3.10. Upgrade the Core and device web dependencies in WSL's user site so ROS launch processes use the compatible versions:

```bash
python3 -m pip install --user --upgrade 'starlette>=1.3.1' 'uvicorn>=0.15' 'websockets>=12'
```

Build from the repository root:

```bash
colcon list --base-paths src
colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_sim_gazebo
source install/setup.bash
```

The simulator-independent backend contracts can be exercised on Humble without starting Gazebo:

```bash
python3 -m unittest discover -s src/sim/common/embodied_nexus_sim/test -v
```

Before launching in WSL, set `ROS_LOCALHOST_ONLY=1` so local ROS nodes use the WSL loopback interface. If `ip -4 -br addr` shows several WSL addresses, set `IGN_IP` to the WSL address reachable from Windows; this makes Gazebo Transport and the spawn client discover the same world service.

```bash
export ROS_LOCALHOST_ONLY=1
export IGN_IP=<WSL IPv4 address reachable from Windows>
ros2 launch embodied_nexus_sim_gazebo sim.launch.py
```

The launch starts Gazebo, the robot model, ros2_control, sensor and world-control bridges, a latched simulation-clock guard, and Core. The sensor bridge starts two seconds after the robot spawn action exits so the bridge can discover Fortress GPU sensor publishers created with the model. The guard pauses the world on a backward clock jump and keeps it paused after manual unpause; restart the launch to clear this fault. The default `gui:=true` opens the WSLg window; use `gui:=false` for a headless run. The launch adds the installed model share directory to Gazebo's resource path and passes `--render-engine ogre` to Fortress so both the GUI and sensor renderer avoid the OGRE2 crash on this WSLg host; see [Gazebo's rendering troubleshooting notes](https://gazebosim.org/docs/fortress/troubleshooting/) and the [WSL2 renderer issue](https://github.com/gazebosim/gz-rendering/issues/662). If a root WSL shell has no `DISPLAY`, set `DISPLAY=:0`, `QT_QPA_PLATFORM=xcb` and a user-owned `XDG_RUNTIME_DIR` with mode `0700` before a GUI launch. Fortress's GPU ray sensor caps each horizontal field of view at 180 degrees, so the scene mounts front and rear 16-line half-scan sensors and the Gazebo-specific relay transforms and merges them into the shared 360-degree point cloud. Cloud xyz values remain in the sensor frame; the relay rotates the rear half into the shared lidar frame without adding a vertical correction. The camera device derives its XYZRGB cloud from depth; the sim profile allows a 250 ms RGB/depth pairing window because the separate Gazebo sensors are phase-offset by about 200 ms.

The default `performance:=balanced` profile uses 320x240 RGB-D at 5 Hz, 5 Hz lidar and no shadows. CameraInfo intrinsics and the Core camera device configuration match this resolution; generated world and configuration files are temporary and removed on launch shutdown. The raw RGB-D payload is one eighth of the original 640x480 / 10 Hz profile. Use `performance:=full` to restore that profile, including shadows. The Bash and PowerShell launch scripts accept the same choice through `EMBODIED_NEXUS_SIM_PERFORMANCE=full`. Physics, control rates and the two 16-line half-scan fields of view are unchanged. The shared model mounts the lidar on top of the rear control tower.

The camera geometry uses Yahboom's currently selected configuration, read over SSH on 2026-10-25: `~/zhaoyang/Arachne/config/hand_eye_camera_extrinsics/current.json` selects `current_effective.json`, stored in the shared `embodied_nexus_sim_description/config/gemini335_extrinsics.json`. Its optical origin in `ee_camera_link` is `(0.0201, 0, 0.05)` m with yaw `-pi/2`; this is the active physical prior, not a newly validated hand-eye solve. Factory intrinsics and the RGB-D baseline come from the installed `embodied_nexus_camera/config/gemini335_factory_calibration.json`, verified against Yahboom's `config/gemini335_intrinsics/factory_calibration.json`. The RGB and depth cameras use separate Gazebo sensors and projection matrices, with the same scaled intrinsics in CameraInfo. The images represent ideal rectified pinhole output (`D=0`); raw optical distortion and sensor noise are not reproduced. Projection support follows [Gazebo Sensors 6's camera implementation](https://github.com/gazebosim/gz-sensors/blob/ign-sensors6/src/CameraSensor.cc). No camera or robot process is started on Yahboom to read these files.

The local Core token defaults to `sim-local-token-change-me`; pass a private `core_token:=...` launch argument to use another value. The simulation profile binds Core to `0.0.0.0:8780` for Windows-to-WSL localhost forwarding. In a Windows PowerShell terminal, set the same token and run the SDK acceptance script from the repository root:

```powershell
$env:EMBODIED_NEXUS_CORE_URL = "http://127.0.0.1:8780"
$env:EMBODIED_NEXUS_CORE_TOKEN = "sim-local-token-change-me"
$env:EMBODIED_NEXUS_IGN_IP = "<the reachable WSL IPv4 used for IGN_IP>"
python src/sim/gazebo/embodied_nexus_sim_gazebo/examples/validate_sim.py
python src/sim/gazebo/embodied_nexus_sim_gazebo/examples/validate_clock_rewind.py
```

The clock-rewind script runs a real Gazebo time-only reset, then checks that the active base operation fails and that a manual unpause is immediately guarded again. The rewind guard intentionally leaves the world paused; stop and restart the WSL launch after this test to clear the fault latch.

The main acceptance script starts, queries and shuts down each configured device independently, then checks base travel and turning, lidar-based obstacle stopping, arm trajectory and velocity commands, arm cancellation, gripper open/close and cancelled relative motion, synchronized RGB/depth frames with intrinsics and depth scale, LaserScan, PointCloud2, and final device shutdown. It also compares fresh wrist-camera color frames around arm motion to verify that the camera observation changes. The clock-rewind script injects `WorldControl.reset.time_only`, verifies an active operation fails, then manually unpauses and checks that the guard pauses again without scene movement. The simulation filter suppresses close self-returns from the lidar mounted above the chassis; the base validation stops at 1.2 m to leave room before contact. Build, test and visual GUI evidence are recorded in [the 2026-10-24 report](acceptance-20261024.md).

Core advertises all five devices but leaves their services stopped until the SDK starts them. `/clock` supplies simulation timestamps; lease expiry, command timeout and feedback age use the host monotonic clock. Restart the launch to reset the world; the first release has no in-process reset API.

Simulated stop feedback, joint position and odometry describe model state only. These values do not represent a hardware E-stop, battery, gripping force or a vendor controller's electrical state. The profile does not claim real transmission calibration, sensor noise fidelity, visual localization, MoveIt planning or a complete grasp task.

The Fortress backend receives base commands as `TwistStamped`, arm/gripper targets as joint trajectories, and sensor images / point clouds through `ros_gz_bridge`. A later simulator can publish the same `/sim/<instance>/` interfaces and select `backend=ros2_sim` without changing device service or SDK APIs.

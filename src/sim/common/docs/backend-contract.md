# Simulator backend contract

An external simulator adapter publishes and consumes standard ROS 2 interfaces below `/sim/<instance>/`:

| Capability | ROS interface |
|---|---|
| Mobile base | `geometry_msgs/TwistStamped`, `nav_msgs/Odometry` |
| Manipulator and gripper | `sensor_msgs/JointState`, `trajectory_msgs/JointTrajectory` |
| Arm and gripper operation targets | `trajectory_msgs/JointTrajectory` |
| Arm action execution | `control_msgs/FollowJointTrajectory` when offered by the adapter |
| RGB-D camera | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` |
| Lidar | `sensor_msgs/PointCloud2` |
| Frames and simulation time | TF and `/clock` |

`backend=ros2_sim` loads through the per-device Python entry-point groups. Per-backend topics and model mappings live in `backend_options`. A future simulator adds its own package and launch profile; it does not alter SDK calls or import private device modules.

# Shared simulator interface

`embodied_nexus_sim` provides the `ros2_sim` backends for the public base, arm and gripper backend contracts. The model package and vendor assets are kept in `third_party/embodied_nexus_sim_description/`. Simulator adapters publish the same ROS 2 control and sensor interfaces under `/sim/<instance>/`; they do not import device internals. Add a simulator under its own `src/sim/<name>/` directory and bind that simulator to these topics and actions.

The common description package contains the adapted robot model and vendor meshes. It has no simulator runtime dependency. Simulation timestamps follow `/clock`; device operation timeouts and sample age use host monotonic time.

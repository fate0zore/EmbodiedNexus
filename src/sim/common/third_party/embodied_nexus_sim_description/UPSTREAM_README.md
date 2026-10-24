# Simulation description sources

The upstream robot description and mounting values originate from `src/arachne_description/` at Arachne commit `41caa564204eb5f6a39200ce2600ed052ab517b0` (MIT). The Aubo i5 meshes and description source originate from `third_party/aubo_description/` in that same Arachne commit; its `package.xml` declares BSD, and that snapshot contains no license text. Scout meshes originate from `third_party/scout_ros2/scout_description/` in that commit; the repository license is Apache-2.0. The original Scout wheel xacro also preserves its upstream BSD notice inline.

The retained file paths are recorded in `third_party/source/`. Meshes are installed under this package's `meshes/` directory. The xacro patch changes package URIs, limits the packaged model to the available MS42DC assets, and parameterizes the `ros2_control` hardware plugin so the shared description has no Gazebo plugin dependency. Fortress plugins and sensors are injected by `embodied_nexus_sim_gazebo`.

The local simulation-only wheel-axis patch parameterizes each Scout wheel joint; when `gazebo_drive_wheels:=true`, positive velocity commands roll the model along `base_link` +X so Gazebo scene truth and controller odometry use the same forward direction. The default upstream axis is unchanged for non-simulation descriptions.

#include <chrono>
#include <memory>
#include <thread>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>

/** @brief Report the closest self and world collision pairs during simulation.
 * @param argc Argument count.
 * @param argv Command-line arguments.
 * @return Zero after shutdown or one when planning scene initialization fails.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("visual_servo_collision_probe",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  auto monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!monitor->getPlanningScene()) {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  monitor->startStateMonitor("/sim/arachne/joint_states");
  monitor->startSceneMonitor("/planning_scene");
  monitor->requestPlanningSceneState("/get_planning_scene");
  rclcpp::WallRate rate(1.0);
  while (rclcpp::ok()) {
    {
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
    collision_detection::DistanceRequest request;
    request.group_name = "manipulator";
    request.enableGroup(scene->getRobotModel());
    request.acm = &scene->getAllowedCollisionMatrix();
    collision_detection::DistanceResult self, world;
    const auto state = monitor->getStateMonitor()->getCurrentState();
    state->updateCollisionBodyTransforms();
    scene->getCollisionEnvUnpadded()->distanceSelf(request, self, *state);
    scene->getCollisionEnv()->distanceRobot(request, world, *state);
    RCLCPP_INFO(node->get_logger(), "self=%.6f (%s,%s) world=%.6f (%s,%s)",
                self.minimum_distance.distance, self.minimum_distance.link_names[0].c_str(),
                self.minimum_distance.link_names[1].c_str(), world.minimum_distance.distance,
                world.minimum_distance.link_names[0].c_str(), world.minimum_distance.link_names[1].c_str());
    }
    rate.sleep();
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}

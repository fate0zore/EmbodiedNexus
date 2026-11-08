#include <cmath>
#include <vector>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>
#include <pluginlib/class_list_macros.hpp>

namespace embodied_nexus_visual_servo {
class PassthroughFilter : public online_signal_smoothing::SmoothingBaseClass {
 public:
  /** @brief Store the expected joint count for simulation-only filtering.
   * @param node ROS node supplied by MoveIt Servo.
   * @param robot_model Robot model supplied by MoveIt Servo.
   * @param num_joints Expected number of arm joints.
   * @return True when at least one joint is configured.
   */
  bool initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                  size_t num_joints) override {
    (void)node;
    (void)robot_model;
    count_ = num_joints;
    return count_ > 0;
  }

  /** @brief Validate finite joint targets while leaving them unchanged.
   * @param positions Joint positions produced by MoveIt Servo.
   * @return True when all expected values are finite.
   */
  bool doSmoothing(std::vector<double>& positions) override {
    if (positions.size() != count_) return false;
    for (double value : positions) if (!std::isfinite(value)) return false;
    return true;
  }

  /** @brief Validate the joint state supplied when Servo resets the filter.
   * @param positions Current joint positions.
   * @return True when the state contains the expected finite values.
   */
  bool reset(const std::vector<double>& positions) override {
    if (positions.size() != count_) return false;
    for (double value : positions) if (!std::isfinite(value)) return false;
    return true;
  }

 private:
  size_t count_ = 0;
};
}  // namespace embodied_nexus_visual_servo

PLUGINLIB_EXPORT_CLASS(embodied_nexus_visual_servo::PassthroughFilter,
                       online_signal_smoothing::SmoothingBaseClass)

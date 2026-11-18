
#include <memory>
#include <string>

#include "roadnet/plugins/edge_cost_functions/start_pose_orientation_scorer.hpp"

namespace roadnet
{

void StartPoseOrientationScorer::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring start pose orientation scorer.");
  name_ = name;
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    getName() + ".orientation_tolerance", rclcpp::ParameterValue(M_PI / 2.0));

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".orientation_weight", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".use_orientation_threshold", rclcpp::ParameterValue(false));

  orientation_tolerance_ = node->get_parameter(getName() + ".orientation_tolerance").as_double();
  orientation_weight_ =
    static_cast<float>(node->get_parameter(getName() + ".orientation_weight").as_double());
  use_orientation_threshold_ =
    node->get_parameter(getName() + ".use_orientation_threshold").as_bool();

  tf_buffer_ = tf_buffer;
}

bool StartPoseOrientationScorer::score(
  const EdgePtr edge,
  const RouteRequest & route_request,
  const EdgeType & edge_type, float & cost)
{
  if (!route_request.use_poses) {
    throw nav2_core::InvalidEdgeScorerUse(
            "Cannot use start pose orientation scorer without start pose specified!");
  }

  if (edge_type == EdgeType::START) {
    double edge_orientation = std::atan2(
      edge->end->coords.y - edge->start->coords.y,
      edge->end->coords.x - edge->start->coords.x);
    double start_orientation = tf2::getYaw(route_request.start_pose.pose.orientation);
    double d_yaw = std::abs(angles::shortest_angular_distance(edge_orientation, start_orientation));

    if (use_orientation_threshold_) {
      if (d_yaw > orientation_tolerance_) {
        return false;
      }
    }

    cost = orientation_weight_ * static_cast<float>(d_yaw);
  }
  return true;
}

std::string StartPoseOrientationScorer::getName()
{
  return name_;
}

}  // namespace roadnet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roadnet::StartPoseOrientationScorer, roadnet::EdgeCostFunction)

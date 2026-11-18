

#include <memory>
#include <string>

#include "roadnet/plugins/route_operations/adjust_speed_limit.hpp"

namespace roadnet
{

void AdjustSpeedLimit::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring Adjust speed limit operation.");
  name_ = name;
  logger_ = node->get_logger();
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".speed_tag", rclcpp::ParameterValue("speed_limit"));
  speed_tag_ = node->get_parameter(getName() + ".speed_tag").as_string();


  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
  std::string topic = node->get_parameter(getName() + ".speed_tag").as_string();

  speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(topic, 1);
  speed_limit_pub_->on_activate();
}

OperationResult AdjustSpeedLimit::perform(
  NodePtr /*node_achieved*/,
  EdgePtr edge_entered,
  EdgePtr /*edge_exited*/,
  const Route & /*route*/,
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const Metadata * /*mdata*/)
{
  OperationResult result;
  if (!edge_entered) {
    return result;
  }

  float speed_limit = 100.0;
  speed_limit = edge_entered->metadata.getValue<float>(speed_tag_, speed_limit);
  RCLCPP_DEBUG(logger_, "Setting speed limit to %.2f%% of maximum.", speed_limit);

  auto msg = std::make_unique<nav2_msgs::msg::SpeedLimit>();
  msg->percentage = true;
  msg->speed_limit = speed_limit;
  speed_limit_pub_->publish(std::move(msg));
  return result;
}

}  // namespace roadnet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roadnet::AdjustSpeedLimit, roadnet::RouteOperation)

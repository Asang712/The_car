
#include <memory>
#include <string>

#include "roadnet/plugins/edge_cost_functions/distance_scorer.hpp"

namespace roadnet
{

void DistanceScorer::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer>/* tf_buffer */,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring distance scorer.");
  name_ = name;

  // Find the tag at high the speed limit information is stored
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".speed_tag", rclcpp::ParameterValue("speed_limit"));
  speed_tag_ = node->get_parameter(getName() + ".speed_tag").as_string();

  // Find the proportional weight to apply, if multiple cost functions
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".weight", rclcpp::ParameterValue(1.0));
  weight_ = static_cast<float>(node->get_parameter(getName() + ".weight").as_double());
}

bool DistanceScorer::score(
  const EdgePtr edge,
  const RouteRequest & /* route_request */,
  const EdgeType & /* edge_type */, float & cost)
{
  // Get the speed limit, if set for an edge
  float speed_val = 1.0f;
  speed_val = edge->metadata.getValue<float>(speed_tag_, speed_val);
  cost = weight_ * hypotf(
    edge->end->coords.x - edge->start->coords.x,
    edge->end->coords.y - edge->start->coords.y) / speed_val;
  return true;
}

std::string DistanceScorer::getName()
{
  return name_;
}

}  // namespace roadnet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roadnet::DistanceScorer, roadnet::EdgeCostFunction)

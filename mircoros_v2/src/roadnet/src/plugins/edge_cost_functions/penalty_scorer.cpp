
#include <memory>
#include <string>

#include "roadnet/plugins/edge_cost_functions/penalty_scorer.hpp"

namespace roadnet
{

void PenaltyScorer::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer>/* tf_buffer */,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring penalty scorer.");
  name_ = name;

  // Find the tag at high the speed limit information is stored
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".penalty_tag", rclcpp::ParameterValue("penalty"));
  penalty_tag_ = node->get_parameter(getName() + ".penalty_tag").as_string();

  // Find the proportional weight to apply, if multiple cost functions
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".weight", rclcpp::ParameterValue(1.0));
  weight_ = static_cast<float>(node->get_parameter(getName() + ".weight").as_double());
}

bool PenaltyScorer::score(
  const EdgePtr edge,
  const RouteRequest & /* route_request */,
  const EdgeType & /* edge_type */, float & cost)
{
  // Get the speed limit, if set for an edge
  float penalty_val = 0.0f;
  penalty_val = edge->metadata.getValue<float>(penalty_tag_, penalty_val);
  cost = weight_ * penalty_val;
  return true;
}

std::string PenaltyScorer::getName()
{
  return name_;
}

}  // namespace roadnet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roadnet::PenaltyScorer, roadnet::EdgeCostFunction)

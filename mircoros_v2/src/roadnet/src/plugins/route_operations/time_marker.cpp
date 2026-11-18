
#include <memory>
#include <string>

#include "roadnet/plugins/route_operations/time_marker.hpp"

namespace roadnet
{

void TimeMarker::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring Adjust speed limit operation.");
  name_ = name;
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".time_tag", rclcpp::ParameterValue("abs_time_taken"));
  time_tag_ = node->get_parameter(getName() + ".time_tag").as_string();
  clock_ = node->get_clock();
  edge_start_time_ = rclcpp::Time(0.0);
}

OperationResult TimeMarker::perform(
  NodePtr /*node_achieved*/,
  EdgePtr edge_entered,
  EdgePtr edge_exited,
  const Route & /*route*/,
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const Metadata * /*mdata*/)
{
  OperationResult result;
  rclcpp::Time now = clock_->now();
  if (!edge_exited || edge_exited->edgeid != curr_edge_) {
    edge_start_time_ = now;
    curr_edge_ = edge_entered ? edge_entered->edgeid : 0;
    return result;
  }

  float dur = static_cast<float>((now - edge_start_time_).seconds());
  if (edge_start_time_.seconds() > 1e-3) {
    edge_exited->metadata.setValue<float>(time_tag_, dur);
  }

  edge_start_time_ = now;
  curr_edge_ = edge_entered ? edge_entered->edgeid : 0;
  return result;
}

}  // namespace roadnet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roadnet::TimeMarker, roadnet::RouteOperation)

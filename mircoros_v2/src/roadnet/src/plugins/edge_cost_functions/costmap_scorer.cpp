
#include <memory>
#include <string>

#include "roadnet/plugins/edge_cost_functions/costmap_scorer.hpp"

namespace roadnet
{

void CostmapScorer::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::shared_ptr<tf2_ros::Buffer>/* tf_buffer */,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring costmap scorer.");
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Find whether to use average or maximum cost values
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".use_maximum", rclcpp::ParameterValue(true));
  use_max_ = static_cast<float>(node->get_parameter(getName() + ".use_maximum").as_bool());

  // Edge is invalid if its in collision
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".invalid_on_collision", rclcpp::ParameterValue(true));
  invalid_on_collision_ =
    static_cast<float>(node->get_parameter(getName() + ".invalid_on_collision").as_bool());

  // Edge is invalid if edge is off the costmap
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".invalid_off_map", rclcpp::ParameterValue(true));
  invalid_off_map_ =
    static_cast<float>(node->get_parameter(getName() + ".invalid_off_map").as_bool());

  // Max cost to be considered valid
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".max_cost", rclcpp::ParameterValue(253.0));
  max_cost_ = static_cast<float>(node->get_parameter(getName() + ".max_cost").as_double());

  // Resolution to check the costmap over (1=every cell, 2=every other cell, etc.)
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".check_resolution", rclcpp::ParameterValue(2));
  check_resolution_ = static_cast<unsigned int>(
    node->get_parameter(getName() + ".check_resolution").as_int());

  // Create costmap subscriber if not the same as the server costmap
  std::string server_costmap_topic = node->get_parameter("costmap_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".costmap_topic",
    rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
  std::string costmap_topic =
    node->get_parameter(getName() + ".costmap_topic").as_string();
  if (costmap_topic != server_costmap_topic) {
    costmap_subscriber_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node, costmap_topic);
    RCLCPP_INFO(node->get_logger(),
      "Using costmap topic: %s instead of server costmap topic: %s for CostmapScorer.",
      costmap_topic.c_str(), server_costmap_topic.c_str());
  } else {
    costmap_subscriber_ = costmap_subscriber;
  }

  // Find the proportional weight to apply, if multiple cost functions
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".weight", rclcpp::ParameterValue(1.0));
  weight_ = static_cast<float>(node->get_parameter(getName() + ".weight").as_double());

  // Collision visualization publisher
  nav2_util::declare_parameter_if_not_declared(
    node, getName() + ".marker_frame", rclcpp::ParameterValue("map"));
  marker_frame_id_ = node->get_parameter(getName() + ".marker_frame").as_string();

  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    getName() + "/collision_markers", rclcpp::QoS(1).transient_local());
}

void CostmapScorer::prepare()
{
  try {
    costmap_ = costmap_subscriber_->getCostmap();
  } catch (...) {
    costmap_.reset();
  }
}

bool CostmapScorer::score(
  const EdgePtr edge,
  const RouteRequest & /* route_request */,
  const EdgeType & /* edge_type */, float & cost)
{
  if (!costmap_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "[CostmapScorer] REJECTED edge[%u] (%.2f,%.2f)->(%.2f,%.2f): No costmap yet received!",
      edge->edgeid,
      edge->start->coords.x, edge->start->coords.y,
      edge->end->coords.x, edge->end->coords.y);
    return false;
  }

  float largest_cost = 0.0, running_cost = 0.0, point_cost = 0.0;
  unsigned int x0, y0, x1, y1, idx = 0;
  std::vector<geometry_msgs::msg::Point> collision_points;
  bool has_collision = false;
  if (!costmap_->worldToMap(edge->start->coords.x, edge->start->coords.y, x0, y0) ||
    !costmap_->worldToMap(edge->end->coords.x, edge->end->coords.y, x1, y1))
  {
    if (invalid_off_map_) {
      // Edge is invalid if it is off the costmap
      RCLCPP_WARN(logger_,
        "[CostmapScorer] REJECTED edge[%u]: edge OFF MAP. start(%.2f,%.2f) end(%.2f,%.2f), "
        "invalid_off_map=%d",
        edge->edgeid,
        edge->start->coords.x, edge->start->coords.y,
        edge->end->coords.x, edge->end->coords.y,
        static_cast<int>(invalid_off_map_));
      return false;
    }
    return true;
  }

  for (nav2_util::LineIterator iter(x0, y0, x1, y1); iter.isValid(); ) {
    point_cost = static_cast<float>(costmap_->getCost(iter.getX(), iter.getY()));
    if (point_cost >= max_cost_ && max_cost_ != 255.0f /*Unknown*/ && invalid_on_collision_) {
      // Edge is invalid if it is in collision or higher than max allowed cost
      double wx = 0.0, wy = 0.0;
      costmap_->mapToWorld(iter.getX(), iter.getY(), wx, wy);
      geometry_msgs::msg::Point collision_pt;
      collision_pt.x = wx;
      collision_pt.y = wy;
      collision_pt.z = 0.0;
      collision_points.push_back(collision_pt);
      has_collision = true;
      RCLCPP_WARN(logger_,
        "[CostmapScorer] REJECTED edge[%u]: COLLISION at cell(%u,%u) world(%.2f,%.2f), "
        "cell_cost=%.0f >= max_cost=%.0f. edge(%.2f,%.2f)->(%.2f,%.2f)",
        edge->edgeid,
        iter.getX(), iter.getY(), wx, wy,
        point_cost, max_cost_,
        edge->start->coords.x, edge->start->coords.y,
        edge->end->coords.x, edge->end->coords.y);
    }

    idx++;
    running_cost += point_cost;
    if (largest_cost < point_cost && point_cost != 255.0) {
      largest_cost = point_cost;
    }

    // Advance the iterator by the check resolution on the edge, pruning to a coarse resolution
    for (unsigned int i = 0; i < check_resolution_; i++) {
      iter.advance();
    }
  }

  if (has_collision) {
    publishCollisionMarkers(collision_points);
    return false;
  }

  if (use_max_) {
    cost = weight_ * largest_cost / max_cost_;
  } else {
    cost = weight_ * running_cost / (static_cast<float>(idx) * max_cost_);
  }

  return true;
}

std::string CostmapScorer::getName()
{
  return name_;
}

void CostmapScorer::publishCollisionMarkers(
  const std::vector<geometry_msgs::msg::Point> & collision_points)
{
  if (!marker_pub_ || collision_points.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  auto now = clock_->now();

  // Clear all previous markers first
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = marker_frame_id_;
  clear_marker.header.stamp = now;
  clear_marker.ns = "costmap_scorer";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  // Add a sphere marker for each collision point
  for (size_t i = 0; i < collision_points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = now;
    marker.ns = "costmap_scorer";
    marker.id = static_cast<int>(i) + 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = collision_points[i];
    marker.pose.position.z = 0.1;  // Slightly above ground for visibility
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    // Orange color for distinction from CollisionMonitor (red)
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;

    marker.lifetime = rclcpp::Duration::from_seconds(180.0);

    marker_array.markers.push_back(marker);

    RCLCPP_INFO(
      logger_, "CostmapScorer collision marker published at world(%.2f, %.2f)",
      collision_points[i].x, collision_points[i].y);
  }

  marker_pub_->publish(marker_array);
}

}  // namespace roadnet

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roadnet::CostmapScorer, roadnet::EdgeCostFunction)


#ifndef NAV2_ROUTE__EDGE_SCORER_HPP_
#define NAV2_ROUTE__EDGE_SCORER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "tf2_ros/buffer.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roadnet/types.hpp"
#include "roadnet/utils.hpp"
#include "roadnet/interfaces/edge_cost_function.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace roadnet
{

/**
 * @class roadnet::EdgeScorer
 * @brief An class to encapsulate edge scoring logic for plugins and different user
 * specified algorithms to influence graph search. It has access to the edge, which
 * in turn has access to the parent and child node of the connection. It also contains
 * action and arbitrary user-defined metadata to enable edge scoring logic based on
 * arbitrary properties of the graph you select (e.g. some regions have a multiplier,
 * some actions are discouraged with higher costs like having to go through a door,
 * edges with reduced speed limits are proportionally less preferred for optimality
 * relative to the distance the edge represents to optimize time to goal)
 */
class EdgeScorer
{
public:
  /**
   * @brief Constructor
   */
  explicit EdgeScorer(
    nav2_util::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber);

  /**
   * @brief Destructor
   */
  ~EdgeScorer() = default;

  /**
   * @brief Score the edge with the set of plugins
   * @param edge Ptr to edge for scoring
   * @param goal_pose Pose Stamped of desired goal
   * @param score of edge
   * @param final_edge whether this edge brings us to the goal or not
   * @return If edge is valid
   */
  bool score(
    const EdgePtr edge, const RouteRequest & route_request,
    const EdgeType & edge_type,
    float & score);

  /**
   * @brief Provide the number of plugisn in the scorer loaded
   * @return Number of scoring plugins
   */
  int numPlugins() const;

protected:
  pluginlib::ClassLoader<EdgeCostFunction> plugin_loader_;
  std::vector<EdgeCostFunction::Ptr> plugins_;
  rclcpp::Logger logger_{rclcpp::get_logger("EdgeScorer")};
};

}  // namespace roadnet

#endif  // NAV2_ROUTE__EDGE_SCORER_HPP_

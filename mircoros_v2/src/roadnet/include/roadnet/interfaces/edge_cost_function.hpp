
#ifndef NAV2_ROUTE__INTERFACES__EDGE_COST_FUNCTION_HPP_
#define NAV2_ROUTE__INTERFACES__EDGE_COST_FUNCTION_HPP_

#include <memory>
#include <string>

#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "roadnet/types.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace roadnet
{

/**
 * @class EdgeCostFunction
 * @brief A plugin interface to score edges during graph search to modify
 * the lowest cost path (e.g. by distance, maximum speed, regions prefer not to travel
 * blocked by occupancy, or using arbitrarily defined user metadata stored in the
 * edge and nodes of interest.)
 */
class EdgeCostFunction
{
public:
  using Ptr = std::shared_ptr<roadnet::EdgeCostFunction>;

  /**
   * @brief Constructor
   */
  EdgeCostFunction() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~EdgeCostFunction() = default;

  /**
   * @brief Configure the scorer, but do not store the node
   * @param parent pointer to user's node
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) = 0;

  /**
   * @brief Main scoring plugin API
   * @param edge The edge pointer to score, which has access to the
   * start/end nodes and their associated metadata and actions
   */
  virtual bool score(
    const EdgePtr edge, const RouteRequest & route_request,
    const EdgeType & edge_type, float & cost) = 0;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  virtual std::string getName() = 0;

  /**
   * @brief Prepare for a new cycle, by resetting state, grabbing data
   * to use for all immediate requests, or otherwise prepare for scoring
   */
  virtual void prepare() {}
};

}  // namespace roadnet

#endif  // NAV2_ROUTE__INTERFACES__EDGE_COST_FUNCTION_HPP_

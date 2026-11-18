
#ifndef NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__COSTMAP_SCORER_HPP_
#define NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__COSTMAP_SCORER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "roadnet/interfaces/edge_cost_function.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

namespace roadnet
{

/**
 * @class CostmapScorer
 * @brief Scores edges by the average or maximum cost found while iterating over the
 * edge's line segment in the global costmap
 */
class CostmapScorer : public EdgeCostFunction
{
public:
  /**
   * @brief Constructor
   */
  CostmapScorer() = default;

  /**
   * @brief destructor
   */
  virtual ~CostmapScorer() = default;

  /**
   * @brief Configure
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber,
    const std::string & name) override;

  /**
   * @brief Main scoring plugin API
   * @param edge The edge pointer to score, which has access to the
   * start/end nodes and their associated metadata and actions
   * @param cost of the edge scored
   * @return bool if this edge is open valid to traverse
   */
  bool score(
    const EdgePtr edge, const RouteRequest & route_request,
    const EdgeType & edge_type, float & cost) override;

  /**
   * @brief Get name of the plugin for parameter scope mapping
   * @return Name
   */
  std::string getName() override;

  /**
   * @brief Prepare for a new cycle, by resetting state, grabbing data
   * to use for all immediate requests, or otherwise prepare for scoring
   */
  void prepare() override;

protected:
  /**
   * @brief Publishes collision markers in RViz2
   * @param collision_points World coordinates of detected collision cells
   */
  void publishCollisionMarkers(const std::vector<geometry_msgs::msg::Point> & collision_points);

  rclcpp::Logger logger_{rclcpp::get_logger("CostmapScorer")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  bool use_max_, invalid_on_collision_, invalid_off_map_;
  float weight_, max_cost_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_{nullptr};
  unsigned int check_resolution_ {1u};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string marker_frame_id_{"map"};
};

}  // namespace roadnet

#endif  // NAV2_ROUTE__PLUGINS__EDGE_COST_FUNCTIONS__COSTMAP_SCORER_HPP_

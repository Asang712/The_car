
#ifndef NAV2_ROUTE__PATH_CONVERTER_HPP_
#define NAV2_ROUTE__PATH_CONVERTER_HPP_

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "roadnet/types.hpp"
#include "roadnet/utils.hpp"
#include "roadnet/corner_smoothing.hpp"

namespace roadnet
{
/**
 * @class roadnet::PathConverter
 * @brief An helper to convert the route into dense paths
 */
class PathConverter
{
public:
  /**
   * @brief A constructor for roadnet::PathConverter
   */
  PathConverter() = default;

  /**
   * @brief A destructor for roadnet::PathConverter
   */
  ~PathConverter() = default;

  /**
   * @brief Configure the object
   * @param node Node to use to get params and create interfaces
   */
  void configure(nav2_util::LifecycleNode::SharedPtr node);

  /**
   * @brief Convert a Route into a dense path
   * @param route Route object
   * @param rerouting_info Re-Route info in case partial path to be populated
   * @param frame Frame ID from planning
   * @param now Time
   * @return Path of the route
   */
  nav_msgs::msg::Path densify(
    const Route & route,
    const ReroutingState & rerouting_info,
    const std::string & frame,
    const rclcpp::Time & now);

  /**
   * @brief Convert an individual edge into a dense line
   * @param x0 X initial
   * @param y0 Y initial
   * @param x1 X final
   * @param y1 Y final
   * @param poses Pose vector reference to populate
   */
  void interpolateEdge(
    float x0, float y0, float x1, float y1,
    std::vector<geometry_msgs::msg::PoseStamped> & poses);

protected:
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("PathConverter")};
  float density_;
  float smoothing_radius_;
  bool smooth_corners_;
};

}  // namespace roadnet

#endif  // NAV2_ROUTE__PATH_CONVERTER_HPP_

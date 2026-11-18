
#ifndef NAV2_ROUTE__GRAPH_SAVER_HPP_
#define NAV2_ROUTE__GRAPH_SAVER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "roadnet/types.hpp"
#include "roadnet/interfaces/graph_file_saver.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

namespace roadnet
{

/**
 * @class roadnet::GraphSaver
 * @brief An object to save graph objects to a file
 */
class GraphSaver
{
public:
  /**
   * @brief A constructor for roadnet::GraphSaver
   * @param node Lifecycle node encapsulated by the GraphSaver
   * @param tf A tf buffer
   * @param frame Coordinate frame that the graph belongs to
   */
  explicit GraphSaver(
    nav2_util::LifecycleNode::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string frame);

  /**
   * @brief A destructor for roadnet::GraphSaver
   */
  ~GraphSaver() = default;

  /**
   * @brief Saves a graph object to a file
   * @param graph Graph to save
   * @param filepath The filepath to the graph data
   * @return bool If successful
   */
  bool saveGraphToFile(
    Graph & graph,
    std::string filepath = "");

  /**
   * @brief Transform the coordinates in the graph to the route frame
   * @param[in/out] graph The graph to be transformed
   * @return True if transformation was successful
 */
  bool transformGraph(Graph & graph);

protected:
  std::string route_frame_, graph_filepath_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("GraphSaver")};

  // Graph Parser
  pluginlib::ClassLoader<GraphFileSaver> plugin_loader_;
  GraphFileSaver::Ptr graph_file_saver_;
  std::string default_plugin_id_;
  std::string plugin_type_;
};

}  // namespace roadnet

#endif  // NAV2_ROUTE__GRAPH_SAVER_HPP_

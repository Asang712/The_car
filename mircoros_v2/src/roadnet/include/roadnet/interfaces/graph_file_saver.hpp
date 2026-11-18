
#ifndef NAV2_ROUTE__INTERFACES__GRAPH_FILE_SAVER_HPP_
#define NAV2_ROUTE__INTERFACES__GRAPH_FILE_SAVER_HPP_

#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "roadnet/types.hpp"

namespace roadnet
{

/**
 * @class GraphFileSaver
 * @brief A plugin interface to parse a file into the graph
 */
class GraphFileSaver
{
public:
  using Ptr = std::shared_ptr<GraphFileSaver>;

  /**
   * @brief Constructor
   */
  GraphFileSaver() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~GraphFileSaver() = default;

  /**
   * @brief Configure the graph file saver, but do not store the node
   * @param parent pointer to user's node
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;

  /**
   * @brief Method to save the graph to the filepath
   * @param graph The graph to save
   * @param filepath The path to save the file to
   * @return true if graph was successfully saved
   */
  virtual bool saveGraphToFile(
    Graph & graph,
    std::string filepath) = 0;
};
}  // namespace roadnet

#endif  // NAV2_ROUTE__INTERFACES__GRAPH_FILE_SAVER_HPP_

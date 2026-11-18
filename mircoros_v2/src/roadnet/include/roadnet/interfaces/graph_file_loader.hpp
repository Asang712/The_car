
#ifndef NAV2_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_
#define NAV2_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_

#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "roadnet/types.hpp"

namespace roadnet
{

/**
 * @class GraphFileLoader
 * @brief A plugin interface to parse a file into the graph
 */
class GraphFileLoader
{
public:
  using Ptr = std::shared_ptr<GraphFileLoader>;

  /**
   * @brief Constructor
   */
  GraphFileLoader() = default;

  /**
   * @brief Virtual destructor
   */
  virtual ~GraphFileLoader() = default;

  /**
   * @brief Configure the graph file loader, but do not store the node
   * @param parent pointer to user's node
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node) = 0;

  /**
   * @brief Method to load the graph from the filepath
   * @param graph The graph to populate
   * @param filepath The file to parse
   * @param idx_map A map translating nodeid's to graph idxs for use in graph modification
   * services and idx-based route planning requests. This is much faster than using a
   * map the full graph data structure.
   * @return true if graph was successfully loaded
   */
  virtual bool loadGraphFromFile(
    Graph & graph,
    GraphToIDMap & graph_to_id_map,
    std::string filepath) = 0;
};
}  // namespace roadnet

#endif  // NAV2_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_

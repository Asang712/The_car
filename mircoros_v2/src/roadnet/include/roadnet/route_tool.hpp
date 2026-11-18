
#ifndef NAV2_ROUTE__ROUTE_TOOL_HPP_
#define NAV2_ROUTE__ROUTE_TOOL_HPP_

#include <ui_route_tool.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "roadnet/graph_loader.hpp"
#include "roadnet/graph_saver.hpp"
#include "roadnet/types.hpp"
#include "roadnet/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"


namespace roadnet
{
/**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

class RouteTool : public rviz_common::Panel
{
  /**
       * This class uses Qt slots and is a subclass of QObject, so it needs
       * the Q_OBJECT macro.
       */
  Q_OBJECT

public:
  /**
           *  QWidget subclass constructors usually take a parent widget
           *  parameter (which usually defaults to 0).  At the same time,
           *  pluginlib::ClassLoader creates instances by calling the default
           *  constructor (with no arguments). Taking the parameter and giving
           *  a default of 0 let's the default constructor work and also let's
           *  someone using the class for something else to pass in a parent
           *  widget as they normally would with Qt.
           */
  explicit RouteTool(QWidget * parent = nullptr);

  void onInitialize() override;

  /**
           *  Now we declare overrides of rviz_common::Panel functions for saving and
           *  loading data from the config file.  Here the data is the topic name.
           */
  virtual void save(rviz_common::Config config) const;
  virtual void load(const rviz_common::Config & config);


  /**
       *  Here we declare some internal slots.
       */

private Q_SLOTS:
  void on_load_button_clicked(void);

  void on_save_button_clicked(void);

  void on_create_button_clicked(void);

  void on_confirm_button_clicked(void);

  void on_delete_button_clicked(void);

  void on_add_node_button_toggled(void);

  void on_edit_node_button_toggled(void);

  /**
       *  Finally, we close up with protected member variables
       */

protected:
  // UI pointer
  std::unique_ptr<Ui::route_tool> ui_;

private:
  void update_route_graph(void);
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<roadnet::GraphLoader> graph_loader_;
  std::shared_ptr<roadnet::GraphSaver> graph_saver_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  roadnet::Graph graph_;
  roadnet::GraphToIDMap graph_to_id_map_;
  roadnet::GraphToIDMap edge_to_node_map_;
  roadnet::GraphToIncomingEdgesMap graph_to_incoming_edges_map_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    graph_vis_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    clicked_point_subscription_;

  unsigned int next_node_id_ = 0;
};
}  // namespace roadnet
#endif  // NAV2_ROUTE__ROUTE_TOOL_HPP_

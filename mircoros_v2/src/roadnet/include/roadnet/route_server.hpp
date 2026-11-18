
#ifndef NAV2_ROUTE__ROUTE_SERVER_HPP_
#define NAV2_ROUTE__ROUTE_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/service_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roadnet/action/compute_route.hpp"
#include "roadnet/action/compute_and_track_route.hpp"
#include "roadnet/action/road_net_to_goal.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "nav2_msgs/msg/route_node.hpp"
#include "nav2_msgs/srv/set_route_graph.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "roadnet/route_exceptions.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "roadnet/types.hpp"
#include "roadnet/utils.hpp"
#include "roadnet/graph_loader.hpp"
#include "roadnet/route_planner.hpp"
#include "roadnet/path_converter.hpp"
#include "roadnet/route_tracker.hpp"
#include "roadnet/goal_intent_extractor.hpp"
#include "roadnet/operations_manager.hpp"

namespace roadnet
{
/**
 * @class roadnet::RouteServer
 * @brief An action server implements a Navigation Route-Graph planner
 * to compliment free-space planning in the Planner Server
 */
class RouteServer : public nav2_util::LifecycleNode
{
public:
  using ComputeRoute = roadnet::action::ComputeRoute;
  using ComputeRouteGoal = ComputeRoute::Goal;
  using ComputeRouteResult = ComputeRoute::Result;
  using ComputeRouteServer = nav2_util::SimpleActionServer<ComputeRoute>;

  using ComputeAndTrackRoute = roadnet::action::ComputeAndTrackRoute;
  using ComputeAndTrackRouteGoal = ComputeAndTrackRoute::Goal;
  using ComputeAndTrackRouteFeedback = ComputeAndTrackRoute::Feedback;
  using ComputeAndTrackRouteResult = ComputeAndTrackRoute::Result;
  using ComputeAndTrackRouteServer = nav2_util::SimpleActionServer<ComputeAndTrackRoute>;

  // FollowPath action client types
  using FollowPath = nav2_msgs::action::FollowPath;
  using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  // ComputeRouteAndFollow action server (uses RoadNetToGoal action type)
  using RoadNetToGoal = roadnet::action::RoadNetToGoal;
  using RoadNetToGoalGoal = RoadNetToGoal::Goal;
  using RoadNetToGoalResult = RoadNetToGoal::Result;
  using RoadNetToGoalFeedback = RoadNetToGoal::Feedback;
  using ComputeRouteAndFollowServer = nav2_util::SimpleActionServer<RoadNetToGoal>;

  /**
   * @brief A constructor for roadnet::RouteServer
   * @param options Additional options to control creation of the node.
   */
  explicit RouteServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for roadnet::RouteServer
   */
  ~RouteServer() = default;

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Main route action server callbacks for computing and tracking a route
   */
  void computeRoute();
  void computeAndTrackRoute();

  /**
   * @brief Compute route, send to FollowPath controller, and track edge progress
   */
  void computeRouteAndFollow();

  /**
   * @brief Abstract method combining finding the starting/ending nodes and the route planner
   * to find the Node locations of interest and route to the goal
   * @param goal The request goal information
   * @param blocked_ids The IDs of blocked graphs / edges
   * @param updated_start_id The nodeID of an updated starting point when tracking
   * a route that corresponds to the next point to route to from to continue progress
   * @return A route of the request
   */
  template<typename GoalT>
  Route findRoute(
    const std::shared_ptr<const GoalT> goal,
    ReroutingState & rerouting_info = ReroutingState());

  /**
   * @brief Main processing called by both action server callbacks to centralize
   * the great deal of shared code between them
   */
  template<typename ActionT>
  void processRouteRequest(
    std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

  /**
   * @brief Find the planning duration of the request and log warnings
   * @param start_time Start of planning time
   * @return Duration of planning time
   */
  rclcpp::Duration findPlanningDuration(const rclcpp::Time & start_time);

  /**
   * @brief Populate result for compute route action
   * @param result Result to populate
   * @param route Route to use to populate result
   * @param path Path to use to populate result
   * @param planning_duration Time to create a valid route
   */
  void populateActionResult(
    std::shared_ptr<ComputeRoute::Result> result,
    const Route & route,
    const nav_msgs::msg::Path & path,
    const rclcpp::Duration & planning_duration);

  /**
   * @brief Populate result for compute and track route action
   * @param result Result to populate
   * @param route Route to use to populate result
   * @param path Path to use to populate result
   * @param planning_duration Time to create a valid route
   */
  void populateActionResult(
    std::shared_ptr<ComputeAndTrackRoute::Result>/*result*/,
    const Route & /*route*/,
    const nav_msgs::msg::Path & /*path*/,
    const rclcpp::Duration & /*planning_duration*/);

  /**
   * @brief The service callback to set a new route graph
   * @param request_header to the service
   * @param request to the service
   * @param response from the service
   */
  void setRouteGraph(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
    std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response);

  /**
   * @brief Clear global costmap to remove residual cost values before planning
   */
  void clearGlobalCostmap();

  /**
   * @brief Log exception warnings, templated by action message type
   * @param goal Goal that failed
   * @param exception Exception message
   */
  template<typename GoalT>
  void exceptionWarning(const std::shared_ptr<const GoalT> goal, const std::exception & ex)
  {
    RCLCPP_WARN(
        get_logger(),
        "Route server failed on original request: Start: [(%0.2f, %0.2f) / %i] Goal: [(%0.2f, %0.2f) / %i]:"
        " \"%s\"",
        goal->start.pose.position.x, goal->start.pose.position.y, goal->start_id,
        goal->goal.pose.position.x, goal->goal.pose.position.y, goal->goal_id, ex.what());
  }
  
  template<typename ActionT>
  bool isRequestValid(std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
  {
    if (!action_server || !action_server->is_server_active())
    {
      RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
      return false;
    }
  
    if (action_server->is_cancel_requested())
    {
      RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling route planning action.");
      action_server->terminate_all();
      return false;
    }
  
    if (graph_.empty())
    {
      RCLCPP_INFO(get_logger(), "No graph set! Aborting request.");
      action_server->terminate_current();
      return false;
    }
  
    return true;
  }
  
  std::shared_ptr<ComputeRouteServer> compute_route_server_;
  std::shared_ptr<ComputeAndTrackRouteServer> compute_and_track_route_server_;
  std::shared_ptr<ComputeRouteAndFollowServer> compute_route_and_follow_server_;

  // FollowPath action client
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

  // NavigateToPose action client for precise goal pose navigation
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Publish the route for visualization
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    graph_vis_publisher_;

  // Set or modify graph
  nav2_util::ServiceServer<nav2_msgs::srv::SetRouteGraph,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>::SharedPtr set_graph_service_;

  // Internal tools
  std::shared_ptr<GraphLoader> graph_loader_;
  std::shared_ptr<RoutePlanner> route_planner_;
  std::shared_ptr<RouteTracker> route_tracker_;
  std::shared_ptr<PathConverter> path_converter_;
  std::shared_ptr<GoalIntentExtractor> goal_intent_extractor_;
  std::unique_ptr<OperationsManager> operations_manager_;

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;

  // Global costmap clear service client
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_client_;

  // State Data
  Graph graph_;
  GraphToIDMap id_to_graph_map_;
  std::string route_frame_, base_frame_;
  double max_planning_time_;
  bool aggregate_blocked_ids_;
};

}  // namespace roadnet

#endif  // NAV2_ROUTE__ROUTE_SERVER_HPP_

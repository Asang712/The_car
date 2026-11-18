
#include "roadnet/route_server.hpp"

using nav2_util::declare_parameter_if_not_declared;
using std::placeholders::_1;
using std::placeholders::_2;

namespace roadnet
{

  RouteServer::RouteServer(const rclcpp::NodeOptions &options)
      : nav2_util::LifecycleNode("route_server", "", options)
  {
  }

  nav2_util::CallbackReturn
  RouteServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    auto node = shared_from_this();
    graph_vis_publisher_ =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "route_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    compute_route_server_ = std::make_shared<ComputeRouteServer>(
        node, "compute_route",
        std::bind(&RouteServer::computeRoute, this),
        nullptr, std::chrono::milliseconds(500), true);

    compute_and_track_route_server_ = std::make_shared<ComputeAndTrackRouteServer>(
        node, "compute_and_track_route",
        std::bind(&RouteServer::computeAndTrackRoute, this),
        nullptr, std::chrono::milliseconds(500), true);

    compute_route_and_follow_server_ = std::make_shared<ComputeRouteAndFollowServer>(
        node, "compute_route_and_follow",
        std::bind(&RouteServer::computeRouteAndFollow, this),
        nullptr, std::chrono::milliseconds(500), true);

    follow_path_client_ = rclcpp_action::create_client<FollowPath>(
        shared_from_this(), "follow_path");

    // Initialize NavigateToPose client for precise goal pose navigation
    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
        shared_from_this(), "navigate_to_pose");

    set_graph_service_ = std::make_shared<nav2_util::ServiceServer<nav2_msgs::srv::SetRouteGraph,
                                                                   std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>>(
        std::string(node->get_name()) + "/set_route_graph",
        node,
        std::bind(
            &RouteServer::setRouteGraph, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    declare_parameter_if_not_declared(
        node, "route_frame", rclcpp::ParameterValue(std::string("map")));
    declare_parameter_if_not_declared(
        node, "base_frame", rclcpp::ParameterValue(std::string("base_link")));
    declare_parameter_if_not_declared(
        node, "max_planning_time", rclcpp::ParameterValue(2.0));

    route_frame_ = node->get_parameter("route_frame").as_string();
    base_frame_ = node->get_parameter("base_frame").as_string();
    max_planning_time_ = node->get_parameter("max_planning_time").as_double();

    // Create costmap subscriber
    nav2_util::declare_parameter_if_not_declared(
        node, "costmap_topic",
        rclcpp::ParameterValue(std::string("global_costmap/costmap_raw")));
    std::string costmap_topic = node->get_parameter("costmap_topic").as_string();
    costmap_subscriber_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node, costmap_topic);

    try
    {
      graph_loader_ = std::make_shared<GraphLoader>(node, tf_, route_frame_);
      if (!graph_loader_->loadGraphFromParameter(graph_, id_to_graph_map_))
      {
        return nav2_util::CallbackReturn::FAILURE;
      }

      goal_intent_extractor_ = std::make_shared<GoalIntentExtractor>();
      goal_intent_extractor_->configure(
          node, graph_, &id_to_graph_map_, tf_, costmap_subscriber_, route_frame_, base_frame_);

      route_planner_ = std::make_shared<RoutePlanner>();
      route_planner_->configure(node, tf_, costmap_subscriber_);

      route_tracker_ = std::make_shared<RouteTracker>();
      route_tracker_->configure(
          node, tf_, costmap_subscriber_, compute_and_track_route_server_, route_frame_, base_frame_);

      // Read aggregate_blocked_ids shared with RouteTracker
      aggregate_blocked_ids_ = node->get_parameter("aggregate_blocked_ids").as_bool();

      // Initialize OperationsManager for computeRouteAndFollow
      operations_manager_ = std::make_unique<OperationsManager>(node, costmap_subscriber_);

      // Create client for clearing global costmap
      clear_costmap_client_ = node->create_client<nav2_msgs::srv::ClearEntireCostmap>(
          "global_costmap/clear_entirely_global_costmap");

      path_converter_ = std::make_shared<PathConverter>();
      path_converter_->configure(node);
    }
    catch (std::exception &e)
    {
      RCLCPP_FATAL(get_logger(), "Failed to configure route server: %s", e.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  RouteServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    compute_route_server_->activate();
    compute_and_track_route_server_->activate();
    compute_route_and_follow_server_->activate();
    graph_vis_publisher_->on_activate();
    graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  RouteServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    compute_route_server_->deactivate();
    compute_and_track_route_server_->deactivate();
    compute_route_and_follow_server_->deactivate();
    graph_vis_publisher_->on_deactivate();
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  RouteServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    compute_route_server_.reset();
    compute_and_track_route_server_.reset();
    compute_route_and_follow_server_.reset();
    follow_path_client_.reset();
    set_graph_service_.reset();
    graph_loader_.reset();
    route_planner_.reset();
    route_tracker_.reset();
    path_converter_.reset();
    goal_intent_extractor_.reset();
    operations_manager_.reset();
    graph_vis_publisher_.reset();
    transform_listener_.reset();
    tf_.reset();
    graph_.clear();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  RouteServer::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  rclcpp::Duration
  RouteServer::findPlanningDuration(const rclcpp::Time &start_time)
  {
    auto cycle_duration = this->now() - start_time;
    if (max_planning_time_ > 0.0 && cycle_duration.seconds() > max_planning_time_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Route planner missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
          1 / max_planning_time_, 1 / cycle_duration.seconds());
    }

    return cycle_duration;
  }

  void RouteServer::populateActionResult(  
      std::shared_ptr<ComputeRoute::Result> result,
      const Route &route,
      const nav_msgs::msg::Path &path,
      const rclcpp::Duration &planning_duration)
  {
    result->route = utils::toMsg(route, route_frame_, this->now());
    result->path = path;
    result->planning_time = planning_duration;
  }

  void RouteServer::populateActionResult(
      std::shared_ptr<ComputeAndTrackRoute::Result> result,
      const Route &,
      const nav_msgs::msg::Path &,
      const rclcpp::Duration &execution_duration)
  {
    result->execution_duration = execution_duration;
  }

  template <typename GoalT>
  Route RouteServer::findRoute(
      const std::shared_ptr<const GoalT> goal,
      ReroutingState &rerouting_info)
  {
    // Find the search boundaries
    auto [start_route, end_route] = goal_intent_extractor_->findStartandGoal(goal);

    // If we're rerouting, use the rerouting start node and pose as the new start
    if (rerouting_info.rerouting_start_id != std::numeric_limits<unsigned int>::max())
    {
      start_route = id_to_graph_map_.at(rerouting_info.rerouting_start_id);
      goal_intent_extractor_->overrideStart(rerouting_info.rerouting_start_pose);
    }

    Route route;
    if (start_route == end_route)
    {
      // Succeed with a single-point route
      route.route_cost = 0.0;
      route.start_node = &graph_.at(start_route);
    }
    else
    {
      // Populate request data (start & goal id, start & goal pose, if set) for routing
      RouteRequest route_request;
      route_request.start_nodeid = start_route;
      route_request.goal_nodeid = end_route;
      route_request.start_pose = goal_intent_extractor_->getStart();
      route_request.goal_pose = goal->goal;
      route_request.use_poses = goal->use_poses;

      // Compute the route via graph-search, returns a node-edge sequence
      route = route_planner_->findRoute(
          graph_, start_route, end_route, rerouting_info.blocked_ids, route_request);
    }

    return goal_intent_extractor_->pruneStartandGoal(route, goal, rerouting_info);
  }

  template <typename ActionT>
  void
  RouteServer::processRouteRequest(
      std::shared_ptr<nav2_util::SimpleActionServer<ActionT>> &action_server)
  {
    auto goal = action_server->get_current_goal();
    auto result = std::make_shared<typename ActionT::Result>();
    ReroutingState rerouting_info;
    auto start_time = this->now();

    try
    {
      while (rclcpp::ok())
      {
        if (!isRequestValid(action_server))
        {
          return;
        }

        if (action_server->is_preempt_requested())
        {
          RCLCPP_INFO(get_logger(), "Computing new preempted route to goal.");
          goal = action_server->accept_pending_goal();
          rerouting_info.reset();
        }

        // Find the route
        Route route = findRoute(goal, rerouting_info);
        RCLCPP_INFO(get_logger(), "Route found with %zu nodes and %zu edges",
                    route.edges.size() + 1u, route.edges.size());
        auto path = path_converter_->densify(route, rerouting_info, route_frame_, this->now());

        if (std::is_same<ActionT, ComputeAndTrackRoute>::value)
        {
          // blocks until re-route requested or task completion, publishes feedback
          switch (route_tracker_->trackRoute(route, path, rerouting_info))
          {
          case TrackerResult::COMPLETED:
            populateActionResult(result, route, path, this->now() - start_time);
            action_server->succeeded_current(result);
            return;
          case TrackerResult::INTERRUPTED:
            // Reroute, cancel, or preempt requested
            break;
          case TrackerResult::EXITED:
            // rclcpp::ok() is false, so just return
            return;
          }
        }
        else
        {
          // Return route if not tracking
          populateActionResult(result, route, path, findPlanningDuration(start_time));
          action_server->succeeded_current(result);
          return;
        }
      }
    }
    catch (nav2_core::NoValidRouteCouldBeFound &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::NO_VALID_ROUTE;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (nav2_core::TimedOut &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::TIMEOUT;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (nav2_core::RouteTFError &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::TF_ERROR;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (nav2_core::NoValidGraph &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::NO_VALID_GRAPH;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (nav2_core::IndeterminantNodesOnGraph &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::INDETERMINANT_NODES_ON_GRAPH;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (nav2_core::InvalidEdgeScorerUse &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::INVALID_EDGE_SCORER_USE;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (nav2_core::OperationFailed &ex)
    {
      // A special case since Operation Failed is only in Compute & Track
      // actions, specifying it to allow otherwise fully shared code
      exceptionWarning(goal, ex);
      result->error_code = ComputeAndTrackRoute::Result::OPERATION_FAILED;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (nav2_core::RouteException &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::UNKNOWN;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
    catch (std::exception &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = ActionT::Result::UNKNOWN;
      result->error_msg = ex.what();
      action_server->terminate_current(result);
    }
  }

  void
  RouteServer::computeRoute()
  {
    RCLCPP_INFO(get_logger(), "Computing route to goal.");
    processRouteRequest(compute_route_server_);
  }

  void
  RouteServer::computeAndTrackRoute()
  {
    RCLCPP_INFO(get_logger(), "Computing and tracking route to goal.");
    processRouteRequest(compute_and_track_route_server_);
  }


  // Compute Route and Follow Path //规划、并执行跟随路径
  void RouteServer::computeRouteAndFollow()
  {
    RCLCPP_INFO(get_logger(), "Computing route and following path to goal.");

    auto goal = compute_route_and_follow_server_->get_current_goal();
    auto result = std::make_shared<RoadNetToGoalResult>();
    auto feedback = std::make_shared<RoadNetToGoalFeedback>();
    ReroutingState rerouting_info;
    auto start_time = this->now();

    try
    {
      Route route;
      nav_msgs::msg::Path path;

      // 重规划触发条件：（障碍物检测）或请求重规划服务（ReroutingService）
      while (rclcpp::ok())
      {
        if (!isRequestValid(compute_route_and_follow_server_))
        {
          return;
        }
        // 检查是否有新的目标被抢占
        if (compute_route_and_follow_server_->is_preempt_requested())
        {
          RCLCPP_INFO(get_logger(), "A new request is received, and the target of the original request is preempted");
          goal = compute_route_and_follow_server_->accept_pending_goal();
          rerouting_info.reset();
        }
        // 规划前清除global_costmap残留代价值，避免影响规划
        clearGlobalCostmap();

        // 计算路网中的可行路径
        route = findRoute(goal, rerouting_info);
        RCLCPP_INFO(get_logger(), "Route found with %zu nodes and %zu edges",
                    route.edges.size() + 1u, route.edges.size());
        path = path_converter_->densify(route, rerouting_info, route_frame_, this->now());

        // 打印路径
        std::string route_str;
        if (route.start_node)
        {
          route_str = std::to_string(route.start_node->nodeid);
        }
        for (const auto &edge : route.edges)
        {
          route_str += " --------->> " + std::to_string(edge->end->nodeid);
        }
        RCLCPP_INFO(get_logger(), "Find route plan: %s", route_str.c_str());

        // Wait for FollowPath action server
        if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
          RCLCPP_ERROR(get_logger(), "FollowPath action server not available!");
          result->error_code = RoadNetToGoalResult::UNKNOWN;
          result->error_msg = "FollowPath action server not available";
          compute_route_and_follow_server_->terminate_current(result);
          return;
        }

        // 发送路径到 FollowPath
        auto follow_goal = FollowPath::Goal();
        follow_goal.path = path;

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        auto goal_handle_future = follow_path_client_->async_send_goal(follow_goal, send_goal_options);

        // Wait for goal acceptance
        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
        {
          RCLCPP_ERROR(get_logger(), "FollowPath goal send timed out!");
          result->error_code = RoadNetToGoalResult::UNKNOWN;
          result->error_msg = "FollowPath goal send timed out";
          compute_route_and_follow_server_->terminate_current(result);
          return;
        }

        auto follow_goal_handle = goal_handle_future.get();
        if (!follow_goal_handle)
        {
          RCLCPP_ERROR(get_logger(), "FollowPath goal was rejected!");
          result->error_code = RoadNetToGoalResult::UNKNOWN;
          result->error_msg = "FollowPath goal rejected by controller";
          compute_route_and_follow_server_->terminate_current(result);
          return;
        }
        RCLCPP_DEBUG(get_logger(), "FollowPath goal accepted, robot is moving!");

        // Get result future for polling
        auto follow_result_future = follow_path_client_->async_get_result(follow_goal_handle);

        // 跟踪路径进度并检测障碍物
        double radius_threshold = 2.0;
        nav2_util::declare_parameter_if_not_declared(
            shared_from_this(), "radius_to_achieve_node",
            rclcpp::ParameterValue(radius_threshold));
        radius_threshold = get_parameter("radius_to_achieve_node").as_double();

        EdgePtr current_edge = nullptr;
        NodePtr next_node = nullptr;
        int edge_idx = -1;
        RouteTrackingState tracking_state;
        tracking_state.last_node = route.start_node;
        tracking_state.next_node = route.start_node;

        // Advance to first edge
        if (!route.edges.empty())
        {
          edge_idx = 0;
          current_edge = route.edges[0];
          next_node = current_edge->end;
          tracking_state.current_edge = current_edge;
          RCLCPP_INFO(get_logger(),
                      "Entering edge:%u  id:%u ---------------------> id:%u",
                      current_edge->edgeid,
                      current_edge->start->nodeid,
                      current_edge->end->nodeid);
        }
        rclcpp::Rate rate(20.0);
        bool follow_completed = false;
        bool route_tracking_done = route.edges.empty();
        bool need_reroute = false;

        while (rclcpp::ok() && !follow_completed && !need_reroute)
        {
          // Check if our action was cancelled
          if (compute_route_and_follow_server_->is_cancel_requested())
          {
            RCLCPP_INFO(get_logger(), "Goal cancelled, cancelling FollowPath...");
            follow_path_client_->async_cancel_goal(follow_goal_handle);
            compute_route_and_follow_server_->terminate_all();
            return;
          }

          // Check if preempted by a new goal
          if (compute_route_and_follow_server_->is_preempt_requested())
          {
            RCLCPP_INFO(get_logger(), "Goal preempted, cancelling FollowPath...");
            follow_path_client_->async_cancel_goal(follow_goal_handle);
            break;  // 跳出内循环，外层循环会处理新目标
          }

          // Check if FollowPath completed
          if (follow_result_future.wait_for(std::chrono::milliseconds(0)) ==
              std::future_status::ready)
          {
            auto wrapped_result = follow_result_future.get();
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
              RCLCPP_INFO(get_logger(), "FollowPath completed successfully!");
            }
            else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED)
            {
              RCLCPP_WARN(get_logger(), "FollowPath was cancelled.");
            }
            else
            {
              RCLCPP_WARN(get_logger(), "FollowPath failed (code=%d).",static_cast<int>(wrapped_result.code));
            }
            follow_completed = true;
            break;
          }

          // 获取机器人当前位置
          geometry_msgs::msg::PoseStamped robot_pose;
          bool got_pose = nav2_util::getCurrentPose(
              robot_pose, *tf_, route_frame_, base_frame_, 0.5);

          // Track edge transitions
          if (!route_tracking_done && next_node && got_pose)
          {
            const double dx = next_node->coords.x - robot_pose.pose.position.x;
            const double dy = next_node->coords.y - robot_pose.pose.position.y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < radius_threshold)
            {
              // Node achieved
              RCLCPP_INFO(get_logger(),
                          "Arrived at node id:%u (%.2f, %.2f)",
                          next_node->nodeid,
                          next_node->coords.x, next_node->coords.y);

              tracking_state.last_node = next_node;
              edge_idx++;

              if (edge_idx < static_cast<int>(route.edges.size()))
              {
                current_edge = route.edges[edge_idx];
                next_node = current_edge->end;
                tracking_state.current_edge = current_edge;
                tracking_state.next_node = next_node;
                tracking_state.route_edges_idx = edge_idx;

                RCLCPP_INFO(get_logger(),
                            "Entering edge:%u  id:%u ---------------------> id:%u",
                            current_edge->edgeid,
                            current_edge->start->nodeid,
                            current_edge->end->nodeid);

                // Publish feedback on edge transition
                feedback->last_node_id = tracking_state.last_node ? tracking_state.last_node->nodeid : 0;
                feedback->next_node_id = next_node ? next_node->nodeid : 0;
                feedback->current_edge_id = current_edge ? current_edge->edgeid : 0;
                feedback->rerouted = false;
                compute_route_and_follow_server_->publish_feedback(feedback);
              }
              else
              {
                // All nodes achieved
                route_tracking_done = true;
                tracking_state.current_edge = nullptr;
                tracking_state.next_node = nullptr;
                RCLCPP_INFO(get_logger(), "All route nodes achieved!");
              }
            }
          }

          // 运行OperationsManager进行障碍物检测和重规划判断
          if (got_pose && operations_manager_)
          {
            bool status_change = (edge_idx >= 0 && tracking_state.last_node != nullptr);
            OperationsResult ops_result = operations_manager_->process(
                status_change, tracking_state, route, robot_pose, rerouting_info);

            // 如果操作触发了重规划请求
            if (ops_result.reroute)
            {
              RCLCPP_WARN(get_logger(), "Rerouting requested by operations (e.g., collision detected)!");
              
              // 保存重规划信息
              if (!ops_result.blocked_ids.empty())
              {
                if (!aggregate_blocked_ids_) {
                  rerouting_info.blocked_ids = ops_result.blocked_ids;
                } else {
                  rerouting_info.blocked_ids.insert(
                      rerouting_info.blocked_ids.end(),
                      ops_result.blocked_ids.begin(), ops_result.blocked_ids.end());
                }
              }

              // 重规划起点设为当前阻塞边的起点节点，使机器人回退到安全节点后重新规划
              NodePtr reroute_start_node = nullptr;
              if (tracking_state.last_node)
              {
                reroute_start_node = tracking_state.last_node;
              }
              else if (tracking_state.current_edge)
              {
                reroute_start_node = tracking_state.current_edge->start;
              }

              if (reroute_start_node)
              {
                rerouting_info.rerouting_start_id = reroute_start_node->nodeid;
                rerouting_info.rerouting_start_pose.pose.position.x = reroute_start_node->coords.x;
                rerouting_info.rerouting_start_pose.pose.position.y = reroute_start_node->coords.y;
                rerouting_info.rerouting_start_pose.header.frame_id = route_frame_;
                rerouting_info.rerouting_start_pose.header.stamp = this->now();
              }
              else
              {
                rerouting_info.rerouting_start_id = std::numeric_limits<unsigned int>::max();
                rerouting_info.rerouting_start_pose = geometry_msgs::msg::PoseStamped();
              }

              // Update so during rerouting we can check if we are continuing on the same edge
              rerouting_info.curr_edge = tracking_state.current_edge;

              // 取消当前FollowPath
              follow_path_client_->async_cancel_goal(follow_goal_handle);
              need_reroute = true;

              // Publish feedback with reroute info
              feedback->last_node_id = tracking_state.last_node ? tracking_state.last_node->nodeid : 0;
              feedback->next_node_id = next_node ? next_node->nodeid : 0;
              feedback->current_edge_id = current_edge ? current_edge->edgeid : 0;
              feedback->operations_triggered = ops_result.operations_triggered;
              feedback->rerouted = true;
              compute_route_and_follow_server_->publish_feedback(feedback);
              break;
            }
          }

          rate.sleep();
        }

        // 如果需要重规划，继续外层循环
        if (need_reroute)
        {
          RCLCPP_INFO(get_logger(), "Rerouting from node %u (%.2f, %.2f)...",
              rerouting_info.rerouting_start_id,
              rerouting_info.rerouting_start_pose.pose.position.x,
              rerouting_info.rerouting_start_pose.pose.position.y);
          continue;
        }

        // 如果FollowPath完成或被取消，跳出循环
        if (follow_completed || compute_route_and_follow_server_->is_cancel_requested())
        {
          break;
        }

        // 如果被抢占，继续外层循环处理新目标
        if (compute_route_and_follow_server_->is_preempt_requested())
        {
          continue;
        }
      }

      // 检查是否被取消
      if (compute_route_and_follow_server_->is_cancel_requested())
      {
        compute_route_and_follow_server_->terminate_all();
        return;
      }

      // If using pose navigation, navigate to goal pose after route following
      if (goal->use_poses)
      {
        RCLCPP_INFO(get_logger(), "Navigating to goal pose...");
        
        // Wait for NavigateToPose action server
        if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
          RCLCPP_WARN(get_logger(), "NavigateToPose action server not available, skipping precise navigation.");
        }
        else
        {
          // Create NavigateToPose goal
          auto nav_goal = NavigateToPose::Goal();
          nav_goal.pose = goal->goal;
          
          // Send goal to NavigateToPose
          auto nav_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
          auto nav_goal_future = nav_to_pose_client_->async_send_goal(nav_goal, nav_goal_options);
          
          // Wait for goal acceptance
          if (nav_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
          {
            RCLCPP_WARN(get_logger(), "NavigateToPose goal send timed out, skipping precise navigation.");
          }
          else
          {
            auto nav_goal_handle = nav_goal_future.get();
            if (!nav_goal_handle)
            {
              RCLCPP_WARN(get_logger(), "NavigateToPose goal was rejected, skipping precise navigation.");
            }
            else
            {
              RCLCPP_INFO(get_logger(), "NavigateToPose goal accepted, robot is moving to precise goal pose!");
              
              // Get result future for polling
              auto nav_result_future = nav_to_pose_client_->async_get_result(nav_goal_handle);
              
              // Wait for NavigateToPose to complete
              rclcpp::Rate nav_rate(20.0);
              bool nav_completed = false;
              
              while (rclcpp::ok() && !nav_completed)
              {
                // Check if our action was cancelled
                if (compute_route_and_follow_server_->is_cancel_requested())
                {
                  RCLCPP_INFO(get_logger(), "Goal cancelled, cancelling NavigateToPose...");
                  nav_to_pose_client_->async_cancel_goal(nav_goal_handle);
                  compute_route_and_follow_server_->terminate_all();
                  return;
                }
                
                // Check if preempted by a new goal
                if (compute_route_and_follow_server_->is_preempt_requested())
                {
                  RCLCPP_INFO(get_logger(), "Goal preempted, cancelling NavigateToPose...");
                  nav_to_pose_client_->async_cancel_goal(nav_goal_handle);
                  // Recursively handle new goal
                  computeRouteAndFollow();
                  return;
                }
                
                // Check if NavigateToPose completed
                if (nav_result_future.wait_for(std::chrono::milliseconds(0)) ==
                    std::future_status::ready)
                {
                  auto nav_result = nav_result_future.get();
                  if (nav_result.code == rclcpp_action::ResultCode::SUCCEEDED)
                  {
                    RCLCPP_INFO(get_logger(), "NavigateToPose completed successfully!");
                  }
                  else if (nav_result.code == rclcpp_action::ResultCode::CANCELED)
                  {
                    RCLCPP_WARN(get_logger(), "NavigateToPose was cancelled.");
                  }
                  else
                  {
                    RCLCPP_WARN(get_logger(), "NavigateToPose failed (code=%d).",
                                static_cast<int>(nav_result.code));
                  }
                  nav_completed = true;
                  break;
                }
                
                nav_rate.sleep();
              }
            }
          }
        }
      }

      // Return result
      result->success = true;
      result->execution_duration = this->now() - start_time;
      RCLCPP_INFO(get_logger(), "========== Route and follow completed! ==========");
      compute_route_and_follow_server_->succeeded_current(result);
    }
    catch (nav2_core::NoValidRouteCouldBeFound &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = RoadNetToGoalResult::NO_VALID_ROUTE;
      result->error_msg = ex.what();
      compute_route_and_follow_server_->terminate_current(result);
    }
    catch (nav2_core::TimedOut &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = RoadNetToGoalResult::TIMEOUT;
      result->error_msg = ex.what();
      compute_route_and_follow_server_->terminate_current(result);
    }
    catch (nav2_core::RouteTFError &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = RoadNetToGoalResult::TF_ERROR;
      result->error_msg = ex.what();
      compute_route_and_follow_server_->terminate_current(result);
    }
    catch (nav2_core::NoValidGraph &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = RoadNetToGoalResult::NO_VALID_GRAPH;
      result->error_msg = ex.what();
      compute_route_and_follow_server_->terminate_current(result);
    }
    catch (nav2_core::IndeterminantNodesOnGraph &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = RoadNetToGoalResult::INDETERMINANT_NODES_ON_GRAPH;
      result->error_msg = ex.what();
      compute_route_and_follow_server_->terminate_current(result);
    }
    catch (std::exception &ex)
    {
      exceptionWarning(goal, ex);
      result->error_code = RoadNetToGoalResult::UNKNOWN;
      result->error_msg = ex.what();
      compute_route_and_follow_server_->terminate_current(result);
    }
  }

  void RouteServer::setRouteGraph(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Request> request,
      std::shared_ptr<nav2_msgs::srv::SetRouteGraph::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Setting new route graph: %s.", request->graph_filepath.c_str());
    graph_.clear();
    id_to_graph_map_.clear();
    try
    {
      if (graph_loader_->loadGraphFromFile(graph_, id_to_graph_map_, request->graph_filepath))
      {
        goal_intent_extractor_->setGraph(graph_, &id_to_graph_map_);
        graph_vis_publisher_->publish(utils::toMsg(graph_, route_frame_, this->now()));
        response->success = true;
        return;
      }
    }
    catch (std::exception &ex)
    {
    }

    RCLCPP_WARN(
        get_logger(),
        "Failed to set new route graph: %s!", request->graph_filepath.c_str());
    response->success = false;
  }

  void RouteServer::clearGlobalCostmap()
  {
    if (!clear_costmap_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
      RCLCPP_DEBUG(get_logger(), "ClearEntireCostmap service not available, skipping costmap clear.");
      return;
    }
  
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto future = clear_costmap_client_->async_send_request(request);
  
    if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready)
    {
      RCLCPP_DEBUG(get_logger(), "Global costmap cleared before planning.");
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "ClearEntireCostmap request timed out.");
    }
  }

} // namespace roadnet

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(roadnet::RouteServer)

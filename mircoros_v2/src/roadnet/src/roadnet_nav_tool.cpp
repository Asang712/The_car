
#include "roadnet/roadnet_nav_tool.hpp"
#include "ui_roadnet_nav_tool.h"

#include <QMessageBox>
#include <QString>
#include <QMetaObject>

namespace roadnet
{

RoadnetNavTool::RoadnetNavTool(QWidget * parent)
: rviz_common::Panel(parent),
  ui_(std::make_unique<Ui::roadnet_nav_tool>()),
  is_navigating_(false)
{
  // Setup UI
  ui_->setupUi(this);

  // Connect signals and slots
  connect(ui_->start_button, &QPushButton::clicked, this, &RoadnetNavTool::on_start_button_clicked);
  connect(ui_->cancel_button, &QPushButton::clicked, this, &RoadnetNavTool::on_cancel_button_clicked);
  connect(ui_->waypoint_mode_radio, &QRadioButton::toggled, this, &RoadnetNavTool::on_navigation_mode_changed);
  connect(ui_->pose_mode_radio, &QRadioButton::toggled, this, &RoadnetNavTool::on_navigation_mode_changed);

  // Initialize UI state
  ui_->navigation_stacked_widget->setCurrentIndex(0);
  ui_->start_button->setEnabled(true);
  ui_->cancel_button->setEnabled(false);
}

RoadnetNavTool::~RoadnetNavTool()
{
  // Cancel any ongoing navigation
  if (is_navigating_) {
    cancelGoal();
  }
}

void RoadnetNavTool::onInitialize()
{
  // Get RViz's ROS node abstraction
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    RCLCPP_ERROR(rclcpp::get_logger("roadnet_nav_tool"), "Unable to get ROS node abstraction");
    return;
  }
  node_ = ros_node_abstraction->get_raw_node();

  // Create action client using RViz's node (which is already being spinned)
  action_client_ = rclcpp_action::create_client<RoadNetToGoal>(
    node_,
    "compute_route_and_follow");

  RCLCPP_INFO(node_->get_logger(), "RoadnetNavTool initialized");
}

void RoadnetNavTool::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void RoadnetNavTool::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void RoadnetNavTool::on_start_button_clicked()
{
  sendGoal();
}

void RoadnetNavTool::on_cancel_button_clicked()
{
  cancelGoal();
}

void RoadnetNavTool::on_navigation_mode_changed()
{
  if (ui_->waypoint_mode_radio->isChecked()) {
    ui_->navigation_stacked_widget->setCurrentIndex(0);
  } else {
    ui_->navigation_stacked_widget->setCurrentIndex(1);
  }
}

void RoadnetNavTool::on_start_pose_mode_changed()
{
  // This slot can be used for additional logic if needed
}

void RoadnetNavTool::sendGoal()
{
  if (is_navigating_) {
    updateStatus("Already navigating!", true);
    return;
  }

  // Check if action server is available
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    updateStatus("Action server not available!", true);
    return;
  }

  auto goal = RoadNetToGoalGoal();

  if (ui_->waypoint_mode_radio->isChecked()) {
    // Waypoint navigation mode
    goal.start_id = ui_->start_id_spin->value();
    goal.goal_id = ui_->goal_id_spin->value();
    goal.use_start = false;
    goal.use_poses = false;

    RCLCPP_INFO(node_->get_logger(), "Sending waypoint navigation goal: start_id=%d, goal_id=%d",
                goal.start_id, goal.goal_id);
  } else {
    // Pose navigation mode
    goal.use_start = false;
    goal.use_poses = true;

    // Set goal pose
    goal.goal.header.frame_id = "map";
    goal.goal.header.stamp = node_->now();
    goal.goal.pose.position.x = ui_->position_x_spin->value();
    goal.goal.pose.position.y = ui_->position_y_spin->value();
    goal.goal.pose.position.z = ui_->position_z_spin->value();
    goal.goal.pose.orientation.x = ui_->orientation_x_spin->value();
    goal.goal.pose.orientation.y = ui_->orientation_y_spin->value();
    goal.goal.pose.orientation.z = ui_->orientation_z_spin->value();
    goal.goal.pose.orientation.w = ui_->orientation_w_spin->value();

    RCLCPP_INFO(node_->get_logger(), "Sending pose navigation goal: position=(%.2f, %.2f, %.2f)",
                goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.position.z);
  }

  // Send goal
  auto send_goal_options = rclcpp_action::Client<RoadNetToGoal>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&RoadnetNavTool::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback = std::bind(&RoadnetNavTool::goalResultCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&RoadnetNavTool::goalFeedbackCallback, this, 
                                                  std::placeholders::_1, std::placeholders::_2);

  auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);

  // Update UI state
  is_navigating_ = true;
  ui_->start_button->setEnabled(false);
  ui_->cancel_button->setEnabled(true);
  updateStatus("Navigating...");
  updateCurrentEdge("None");
}

void RoadnetNavTool::cancelGoal()
{
  if (!is_navigating_ || !current_goal_handle_) {
    return;
  }

  action_client_->async_cancel_goal(current_goal_handle_);
  updateStatus("Cancelling navigation...");
}

void RoadnetNavTool::goalResponseCallback(RoadNetToGoalClient::GoalHandle::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    // Update UI state in main thread
    QMetaObject::invokeMethod(this, [this]() {
      is_navigating_ = false;
      ui_->start_button->setEnabled(true);
      ui_->cancel_button->setEnabled(false);
      updateStatus("Goal rejected by server!", true);
    });
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
  current_goal_handle_ = goal_handle;
}

void RoadnetNavTool::goalResultCallback(const RoadNetToGoalClient::WrappedResult & result)
{
  // Update UI state in main thread
  QMetaObject::invokeMethod(this, [this, result]() {
    is_navigating_ = false;
    ui_->start_button->setEnabled(true);
    ui_->cancel_button->setEnabled(false);

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result.result->success) {
          updateStatus("Navigation succeeded!");
        } else {
          std::string error_msg = "Navigation failed: " + result.result->error_msg;
          updateStatus(error_msg, true);
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        updateStatus("Navigation aborted!", true);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        updateStatus("Navigation cancelled!");
        break;
      default:
        updateStatus("Unknown result code!", true);
        break;
    }

    current_goal_handle_.reset();
    updateCurrentEdge("None");
  });
}

void RoadnetNavTool::goalFeedbackCallback(
  RoadNetToGoalClient::GoalHandle::SharedPtr,
  const std::shared_ptr<const RoadNetToGoalFeedback> feedback)
{
  // Update UI in main thread
  QMetaObject::invokeMethod(this, [this, feedback]() {
    // Display edge tracking information
    unsigned int last_id = feedback->last_node_id;
    unsigned int next_id = feedback->next_node_id;
    unsigned int edge_id = feedback->current_edge_id;
    bool rerouted = feedback->rerouted;

    std::string edge_info = "Last: " + std::to_string(last_id) +
      " -> Next: " + std::to_string(next_id) +
      " (Edge: " + std::to_string(edge_id) + ")";
    if (rerouted) {
      edge_info += " [REROUTED]";
    }
    updateCurrentEdge(edge_info);
  });
}

void RoadnetNavTool::updateStatus(const std::string & message, bool is_error)
{
  QString q_message = QString::fromStdString(message);
  ui_->status_label->setText(q_message);

  if (is_error) {
    ui_->status_label->setStyleSheet("QLabel { color: red; }");
  } else {
    ui_->status_label->setStyleSheet("QLabel { color: green; }");
  }
}

void RoadnetNavTool::updateCurrentEdge(const std::string & edge_info)
{
  QString q_edge_info = QString::fromStdString(edge_info);
  ui_->current_edge_label->setText(q_edge_info);
}

}  // namespace roadnet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(roadnet::RoadnetNavTool, rviz_common::Panel)

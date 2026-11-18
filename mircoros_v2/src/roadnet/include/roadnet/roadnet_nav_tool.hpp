// Copyright 2024 John Chrosniak
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROADNET_NAV_TOOL_HPP_
#define ROADNET_NAV_TOOL_HPP_

#include <memory>
#include <string>

#include "rviz_common/panel.hpp"
#include "rviz_common/display_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "roadnet/action/road_net_to_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Forward declaration of UI class
namespace Ui
{
class roadnet_nav_tool;
}

namespace roadnet
{
class RoadnetNavTool : public rviz_common::Panel
{
  Q_OBJECT

public:
  using RoadNetToGoal = roadnet::action::RoadNetToGoal;
  using RoadNetToGoalGoal = RoadNetToGoal::Goal;
  using RoadNetToGoalResult = RoadNetToGoal::Result;
  using RoadNetToGoalFeedback = RoadNetToGoal::Feedback;
  using RoadNetToGoalClient = rclcpp_action::Client<RoadNetToGoal>;

  explicit RoadnetNavTool(QWidget * parent = nullptr);
  ~RoadnetNavTool() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void on_start_button_clicked();
  void on_cancel_button_clicked();
  void on_navigation_mode_changed();
  void on_start_pose_mode_changed();

private:
  void sendGoal();
  void cancelGoal();
  void goalResponseCallback(RoadNetToGoalClient::GoalHandle::SharedPtr goal_handle);
  void goalResultCallback(const RoadNetToGoalClient::WrappedResult & result);
  void goalFeedbackCallback(
    RoadNetToGoalClient::GoalHandle::SharedPtr,
    const std::shared_ptr<const RoadNetToGoalFeedback> feedback);
  void updateStatus(const std::string & message, bool is_error = false);
  void updateCurrentEdge(const std::string & edge_info);

  std::unique_ptr<Ui::roadnet_nav_tool> ui_;
  rclcpp::Node::SharedPtr node_;
  RoadNetToGoalClient::SharedPtr action_client_;
  RoadNetToGoalClient::GoalHandle::SharedPtr current_goal_handle_;

  bool is_navigating_;
};

}  // namespace roadnet

#endif  // ROADNET_NAV_TOOL_HPP_

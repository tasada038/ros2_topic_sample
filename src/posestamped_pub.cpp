// Copyright (c) 2023 Takumi Asada
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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

class GoalPoseComponent : public rclcpp::Node
{
public:
  GoalPoseComponent()
  : Node("goal_pose_publisher")
  {
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, 
      std::bind(&GoalPoseComponent::goalPoseCallback, this, std::placeholders::_1));
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(), "Received Goal Pose: (%f, %f)",
      msg->pose.position.x, msg->pose.position.y);

    // Publish the received coordinates again
    publisher_->publish(*msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPoseComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

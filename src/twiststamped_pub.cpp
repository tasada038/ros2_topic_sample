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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <random>

class TwistStampedPubComponent : public rclcpp::Node
{
public:
  TwistStampedPubComponent()
  : Node("random_twist_stamped_publisher")
  {
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/random_twist_stamped", 10);

    // Linear velocity limits
    double linear_x_min = -1.0;
    double linear_x_max = 1.0;
    double linear_y_min = -1.0;
    double linear_y_max = 1.0;
    double linear_z_min = -1.0;
    double linear_z_max = 1.0;

    // Angular velocity limits
    double angular_x_min = -1.0;
    double angular_x_max = 1.0;
    double angular_y_min = -1.0;
    double angular_y_max = 1.0;
    double angular_z_min = -1.0;
    double angular_z_max = 1.0;

    // Create aliases for shorter names
    using RealDist = std::uniform_real_distribution<double>;

    linear_x_dist_ = RealDist(linear_x_min, linear_x_max);
    linear_y_dist_ = RealDist(linear_y_min, linear_y_max);
    linear_z_dist_ = RealDist(linear_z_min, linear_z_max);
    angular_x_dist_ = RealDist(angular_x_min, angular_x_max);
    angular_y_dist_ = RealDist(angular_y_min, angular_y_max);
    angular_z_dist_ = RealDist(angular_z_min, angular_z_max);

    timer_ = 
      create_wall_timer(
      std::chrono::seconds(1), 
      std::bind(&TwistStampedPubComponent::publishRandomTwistStamped, this));
  }

private:
  void publishRandomTwistStamped()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "sensor_frame";
    msg.twist.linear.x = linear_x_dist_(random_engine_);
    msg.twist.linear.y = linear_y_dist_(random_engine_);
    msg.twist.linear.z = linear_z_dist_(random_engine_);
    msg.twist.angular.x = angular_x_dist_(random_engine_);
    msg.twist.angular.y = angular_y_dist_(random_engine_);
    msg.twist.angular.z = angular_z_dist_(random_engine_);

    RCLCPP_INFO(get_logger(), "Publishing Random TwistStamped");

    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::default_random_engine random_engine_;
  std::uniform_real_distribution<double> linear_x_dist_;
  std::uniform_real_distribution<double> linear_y_dist_;
  std::uniform_real_distribution<double> linear_z_dist_;
  std::uniform_real_distribution<double> angular_x_dist_;
  std::uniform_real_distribution<double> angular_y_dist_;
  std::uniform_real_distribution<double> angular_z_dist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistStampedPubComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

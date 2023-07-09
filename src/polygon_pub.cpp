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
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>

class PolygonPubComponent : public rclcpp::Node
{
public:
  PolygonPubComponent()
    : Node("polygon_stamped_publisher")
  {
    publisher_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/polygon_stamped_data", 10);

    // ポリゴンの各頂点を作成
    geometry_msgs::msg::Point32 p1;
    p1.x = 0.0;
    p1.y = 1.0;
    p1.z = 0.0;

    geometry_msgs::msg::Point32 p2;
    p2.x = 1.0;
    p2.y = 1.0;
    p2.z = 0.0;

    geometry_msgs::msg::Point32 p3;
    p3.x = 1.0;
    p3.y = 2.0;
    p3.z = 0.0;

    geometry_msgs::msg::Point32 p4;
    p4.x = 0.0;
    p4.y = 2.0;
    p4.z = 0.0;

    polygon_msg_.header.frame_id = "sensor_frame";
    polygon_msg_.polygon.points.push_back(p1);
    polygon_msg_.polygon.points.push_back(p2);
    polygon_msg_.polygon.points.push_back(p3);
    polygon_msg_.polygon.points.push_back(p4);

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PolygonPubComponent::publishPolygonStamped, this));
  }

private:
  void publishPolygonStamped()
  {
    polygon_msg_.header.stamp = now();

    RCLCPP_INFO(get_logger(), "Publishing PolygonStamped");

    publisher_->publish(polygon_msg_);
  }

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PolygonStamped polygon_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PolygonPubComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

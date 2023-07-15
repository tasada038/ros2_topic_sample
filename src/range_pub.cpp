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
#include <sensor_msgs/msg/range.hpp>

class RangePubComponent : public rclcpp::Node
{
public:
  RangePubComponent()
  : Node("range_publisher")
  {
    publisher_ = create_publisher<sensor_msgs::msg::Range>("/range_data", 10);

    min_range_ = 0.1;  // min range [m]
    max_range_ = 5.0; // max range [m]
    range_ = min_range_;

    timer_ = 
      create_wall_timer(std::chrono::seconds(1), std::bind(&RangePubComponent::publishRange, this));
  }

private:
  void publishRange()
  {
    sensor_msgs::msg::Range msg;
    msg.header.frame_id = "sensor_frame";
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.7854;  // 45 degree
    msg.min_range = min_range_;
    msg.max_range = max_range_;
    msg.range = range_;

    RCLCPP_INFO(get_logger(), "Publishing range: %f", msg.range);

    publisher_->publish(msg);

    range_ += 0.1;
    if (range_ > max_range_) {
      range_ = min_range_;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  float min_range_;
  float max_range_;
  float range_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RangePubComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

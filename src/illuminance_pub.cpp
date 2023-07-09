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
#include <sensor_msgs/msg/illuminance.hpp>

#include <random>

class IlluminancePubComponent : public rclcpp::Node
{
public:
  IlluminancePubComponent()
    : Node("random_illuminance_publisher")
  {
    publisher_ = create_publisher<sensor_msgs::msg::Illuminance>("/illuminance_data", 10);

    std::random_device rd;
    gen_.seed(rd());
    dist_ = std::uniform_real_distribution<double>(0.0, 1000.0); // limit
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&IlluminancePubComponent::publishRandomIlluminance, this));
  }

private:
  void publishRandomIlluminance()
  {
    sensor_msgs::msg::Illuminance msg;
    msg.header.frame_id = "sensor_frame";
    msg.header.stamp = now();
    msg.illuminance = dist_(gen_); // random illuminance

    RCLCPP_INFO(get_logger(), "Publishing Illuminance: %f lx", msg.illuminance);

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mt19937 gen_;
  std::uniform_real_distribution<double> dist_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IlluminancePubComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

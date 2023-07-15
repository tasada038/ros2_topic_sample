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
#include <std_msgs/msg/float32.hpp>

class Float32PubComponent : public rclcpp::Node
{
public:
  Float32PubComponent()
  : Node("float32_pub")
  {
    publisher_ = create_publisher<std_msgs::msg::Float32>("/data", 10);
    timer_ =
      create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Float32PubComponent::publishData, this));
  }

private:
  void publishData()
  {
    std_msgs::msg::Float32 msg;
    msg.data = count_;

    RCLCPP_INFO(get_logger(), "Publishing: %f", msg.data);

    publisher_->publish(msg);
    count_++;

    if (count_ > count_limit_) {
      count_ = 0.0;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float count_ = 0;
  float count_limit_ = 40;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Float32PubComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

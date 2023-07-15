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
#include <std_msgs/msg/string.hpp>

#include <random>
#include <vector>
#include <string>

class StringPubComponent : public rclcpp::Node
{
public:
  StringPubComponent()
  : Node("random_string_publisher")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/string_topic", 10);

    rosWords_ = {
      "ROS",
      "Publisher",
      "Subscriber",
      "Service"
      "Node"
    };

    std::random_device rd;
    gen_.seed(rd());
    dist_ = std::uniform_int_distribution<int>(0, rosWords_.size() - 1);

    timer_ =
      create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&StringPubComponent::publishRandomString, this));
  }

private:
  void publishRandomString()
  {
    std_msgs::msg::String msg;
    msg.data = rosWords_[dist_(gen_)];

    RCLCPP_INFO(get_logger(), "Publishing: %s", msg.data.c_str());

    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> rosWords_;
  std::mt19937 gen_;
  std::uniform_int_distribution<int> dist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StringPubComponent>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

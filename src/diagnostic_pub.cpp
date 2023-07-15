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
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

class DiagnosticNode : public rclcpp::Node
{
public:
  DiagnosticNode()
  : Node("diagnostic_node")
    , diagnostic_updater_(this)
  {
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    subscription_ = create_subscription<std_msgs::msg::Float32>(
      "/data", 10, std::bind(&DiagnosticNode::dataCallback, this, std::placeholders::_1));

    diagnostic_updater_.setHardwareID("MyDevice");
    diagnostic_updater_.add(
      "Data Diagnostic", 
      std::bind(&DiagnosticNode::diagnosticCallback, this, std::placeholders::_1));
  }

private:
  void dataCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    data_value_ = msg->data;
    diagnostic_updater_.force_update();
  }

  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (data_value_ < 10.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Data is within the normal range");
    } else if (data_value_ < 20.0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Data is slightly high");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Data is too high");
    }

    diagnostic_msgs::msg::DiagnosticStatus status_msg;
    status_msg.level = stat.level;
    status_msg.name = stat.name;
    status_msg.message = stat.message;
    status_msg.values = stat.values;

    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = "data_value";
    key_value.value = std::to_string(data_value_);
    status_msg.values.push_back(key_value);

    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    diagnostic_array.header.stamp = now();
    diagnostic_array.status.push_back(status_msg);

    publisher_->publish(diagnostic_array);
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  diagnostic_updater::Updater diagnostic_updater_;
  float data_value_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagnosticNode>());
  rclcpp::shutdown();
  return 0;
}

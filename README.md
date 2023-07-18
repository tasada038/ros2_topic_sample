# ros2_topic_sample

[![Build and run ROS2 tests](https://github.com/tasada038/ros2_topic_sample/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/tasada038/ros2_topic_sample/actions/workflows/ros2_ci.yml)
[![GitHub license](https://img.shields.io/github/license/tasada038/ros2_topic_sample.svg)](https://github.com/tasada038/ros2_topic_sample/blob/master/LICENSE)

ROS 2 topic sample package

## Supported ROS 2 distributions

[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

## Requirements
- Ubuntu OS PC
  - Ubuntu 22.04 Humble

## Usage

```
ros2 launch ros2_topic_sample data_publishers.launch.py 
```

![ros2 topic img](/img/ros2_topic_sample.png)

## ROS 2 Topic data

| Topic data             | msgs Type                           |
| -------------          | -------------                       |
| /data                  | std_msgs::msg::Float32              |
| /illuminance_data      | sensor_msgs/msg/Illuminance         |
| /range_data            | sensor_msgs/msg/Range               |
| string_topic           | std_msgs/msg/String                 |
| /polygon_stamped_data  | geometry_msgs/msg/PolygonStamped    |
| /random_twist_stamped  | geometry_msgs/msg/TwistStamped      |
| /diagnostics           | diagnostic_msgs/msg/DiagnosticArray |
| /move_base_simple/goal | geometry_msgs::PoseStamped          |

## License
This repository is licensed under the Apache Software License 2.0, see LICENSE.

[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html

[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/

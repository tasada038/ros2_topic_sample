cmake_minimum_required(VERSION 3.8)
project(ros2_topic_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# diagnostic_pub
add_executable(diagnostic_pub src/diagnostic_pub.cpp)
ament_target_dependencies(diagnostic_pub
  rclcpp
  diagnostic_msgs
  diagnostic_updater
)
target_link_libraries(diagnostic_pub)

# float32_pub
add_executable(float32_pub src/float32_pub.cpp)
ament_target_dependencies(float32_pub
  rclcpp
  std_msgs
)
target_link_libraries(float32_pub)

# illuminance_pub
add_executable(illuminance_pub src/illuminance_pub.cpp)
ament_target_dependencies(illuminance_pub
  rclcpp
  sensor_msgs
)
target_link_libraries(illuminance_pub)

# polygon_pub
add_executable(polygon_pub src/polygon_pub.cpp)
ament_target_dependencies(polygon_pub
  rclcpp
  geometry_msgs
)
target_link_libraries(polygon_pub)

# posestamped_pub
add_executable(posestamped_pub src/posestamped_pub.cpp)
ament_target_dependencies(posestamped_pub
  rclcpp
  geometry_msgs
)
target_link_libraries(posestamped_pub)

# string_pub
add_executable(string_pub src/string_pub.cpp)
ament_target_dependencies(string_pub
  rclcpp
  std_msgs
)
target_link_libraries(string_pub)

# range_pub
add_executable(range_pub src/range_pub.cpp)
ament_target_dependencies(range_pub
  rclcpp
  sensor_msgs
)
target_link_libraries(range_pub)

# twiststamped_pub
add_executable(twiststamped_pub src/twiststamped_pub.cpp)
ament_target_dependencies(twiststamped_pub
  rclcpp
  geometry_msgs
)
target_link_libraries(twiststamped_pub)

# Install Cpp executables
install(TARGETS
  diagnostic_pub
  float32_pub
  illuminance_pub
  polygon_pub
  posestamped_pub
  string_pub
  range_pub
  twiststamped_pub
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(call_m_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#add_executable(bot_control_driver_node src/bot_control_driver.cpp src/servo3moog.cpp src/func.cpp src/kinema.cpp)
#ament_target_dependencies(bot_control_driver_node rclcpp geometry_msgs nav_msgs rosgraph_msgs)

add_executable(camera_control_driver_node src/camera_control_driver.cpp)
ament_target_dependencies(camera_control_driver_node rclcpp dynamixel_sdk_custom_interfaces dynamixel_sdk)

add_executable(sonar_range_node src/sonar_range.cpp)
ament_target_dependencies(sonar_range_node rclcpp sensor_msgs)

# Install directories
install(
  DIRECTORY src launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable
install(TARGETS
  #bot_control_driver_node
  camera_control_driver_node
  sonar_range_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

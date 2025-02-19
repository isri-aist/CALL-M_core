cmake_minimum_required(VERSION 3.5)
project(call_m_teleoperation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Add executables
add_executable(joystick_control_node src/joystick_control.cpp)
ament_target_dependencies(joystick_control_node rclcpp geometry_msgs dynamixel_sdk_custom_interfaces lifecycle_msgs)

add_executable(keyboard_control_node src/keyboard_control.cpp)
ament_target_dependencies(keyboard_control_node rclcpp geometry_msgs dynamixel_sdk_custom_interfaces)

# Install directories
install(
  DIRECTORY src launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable
install(TARGETS
  joystick_control_node
  keyboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(call_m_simulation)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs)

# Add executable
add_executable(simu_bot_driver_node src/simu_bot_driver.cpp)
ament_target_dependencies(simu_bot_driver_node rclcpp std_msgs geometry_msgs dynamixel_sdk dynamixel_sdk_custom_interfaces)

add_executable(simu_odometry_node src/simu_odometry.cpp)
ament_target_dependencies(simu_odometry_node rclcpp geometry_msgs gazebo_msgs nav_msgs rosgraph_msgs)

# Install directories
install(
  DIRECTORY src launch description config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable
install(TARGETS
  simu_bot_driver_node
  simu_odometry_node
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

cmake_minimum_required(VERSION 3.5)
project(c_pkg)

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
find_package(rosidl_default_generators REQUIRED)


### Custom Interfaces 
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/StateVector.msg"
        )


#keyboard_control_node
add_executable(keyboard_control_node src/keyboard_control.cpp src/func.cpp src/kinema.cpp)
ament_target_dependencies(keyboard_control_node rclcpp std_msgs rosidl_default_generators)

#joystick_control_driver_node
add_executable(joystick_control_node src/joystick_control.cpp src/servo3moog.cpp src/func.cpp src/kinema.cpp)
ament_target_dependencies(joystick_control_node rclcpp std_msgs rosidl_default_generators)

#bot_control_driver_node
add_executable(bot_control_driver_node src/bot_control_driver.cpp src/servo3moog.cpp src/func.cpp src/kinema.cpp)
ament_target_dependencies(bot_control_driver_node rclcpp std_msgs rosidl_default_generators)

#link interfaces to nodes
rosidl_target_interfaces(keyboard_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(joystick_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(bot_control_driver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

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

#install nodes
install(TARGETS
  keyboard_control_node
  bot_control_driver_node
  joystick_control_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

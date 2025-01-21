cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)  # Suppress CMP0074 warning

project(call_m_arm_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(call_m_custom_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(controller_manager REQUIRED)
find_package(octomap REQUIRED)

# System dependencies are found with CMake's conventions
find_library(GPD_LIB NAMES gpd PATHS /home/jrluser/arm_and_call_m_ws/src/CALL-M_core/robot_ws_ros2/install/gpd/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)

if (NOT GPD_LIB)
  message(FATAL_ERROR "Library GPD not found")
else()
  message(STATUS "Library GPD found in ${GPD_LIB}")
endif()

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_pytest REQUIRED)
  add_launch_test(test/test_gazebo.py
    TIMEOUT
      180
  )
endif()

include_directories(include ${PCL_INCLUDE_DIRS})

# Include every Python script within the src and scripts directory
file(GLOB PYTHON_CLASSES src/*.py)
file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*.py)
install(PROGRAMS 
  ${PYTHON_CLASSES} 
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Find all .cpp files in the src directory and subdirectories
file(GLOB_RECURSE CPP_SOURCES src/*.cpp)

# Create an executable for each .cpp file
foreach(CPP_FILE ${CPP_SOURCES})
  get_filename_component(EXECUTABLE_NAME ${CPP_FILE} NAME_WE)
  add_executable(${EXECUTABLE_NAME} ${CPP_FILE})
  ament_target_dependencies(${EXECUTABLE_NAME} 
  rclcpp 
  sensor_msgs 
  pcl_conversions 
  PCL 
  std_srvs 
  tf2_ros 
  tf2_geometry_msgs
  tf2_eigen
  moveit
  moveit_core
  moveit_ros_planning_interface
  call_m_custom_msgs
  example_interfaces
  OpenCV
  octomap
  controller_manager
  )
  target_include_directories(${EXECUTABLE_NAME} PRIVATE ${PCL_INCLUDE_DIRS})

  target_link_libraries(${EXECUTABLE_NAME} ${GPD_LIB})

  install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()

# Install multiple directories in one line
install(DIRECTORY
  launch
  config
  worlds
  urdf
  rviz
  src
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

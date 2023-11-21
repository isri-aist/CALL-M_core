# Install script for directory: /home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/nav2_simu

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/install/nav2_simu")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu" TYPE DIRECTORY FILES
    "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/nav2_simu/src"
    "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/nav2_simu/launch"
    "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/nav2_simu/rviz"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/nav2_simu")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/nav2_simu")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu/environment" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu/environment" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_index/share/ament_index/resource_index/packages/nav2_simu")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu/cmake" TYPE FILE FILES
    "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_core/nav2_simuConfig.cmake"
    "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/ament_cmake_core/nav2_simuConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nav2_simu" TYPE FILE FILES "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/src/nav2_simu/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jrlintern/Desktop/CNRS_AIST_Work/CNRS_AIST_Mobile_Shopping_Robot/robot_ws_ros2/build/nav2_simu/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
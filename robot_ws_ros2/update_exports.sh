#update NUC packages
scp -r exports/call_m_NUC_pkgs/ jrluser@150.18.226.30:/home/jrluser/call_m_workspace/robot_ws_ros2/src/

#update Jetson packages
scp -r exports/call_m_JETSON_pkgs/ jrluser@150.18.226.22:/home/jrluser/call_m_workspace/robot_ws_ros2/src/

#update NUC packages
scp -r exports/call_m_NUC_pkgs/ jrluser@192.168.11.11:/home/jrluser/call_m_workspace/robot_ws_ros2/src/

#update Jetson packages
scp -r exports/call_m_JETSON_pkgs/ jrluser@192.168.11.10:/home/jrluser/call_m_workspace/robot_ws_ros2/src/

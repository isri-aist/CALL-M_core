#update NUC packages
scp -r exports/call_m_NUC_pkgs/ jrluser@callm01c.local:/home/jrluser/call_m_workspace/robot_ws_ros2/src/

#update Jetson packages
scp -r exports/call_m_JETSON_pkgs/ jrluser@callm01v.local:/home/jrluser/call_m_workspace/robot_ws_ros2/src/

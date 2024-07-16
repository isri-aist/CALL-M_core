rm -r exports/
mkdir exports/
touch exports/COLCON_IGNORE
mkdir exports/call_m_JETSON_pkgs/ exports/call_m_NUC_pkgs/

#NUC packages
#export files for 'call_m_hardware/call_m_drivers' NUC version
mkdir exports/call_m_NUC_pkgs/call_m_hardware/
mkdir exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/
mkdir exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/src
mkdir exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/config
mkdir exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/launch
cp -r src/call_m_hardware/call_m_drivers/src/ exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/
cp src/call_m_hardware/call_m_drivers/launch/bot_NUC.launch.py exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/launch/
cp src/call_m_hardware/call_m_drivers/CMakeLists.txt exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/
cp src/call_m_hardware/call_m_drivers/package.xml exports/call_m_NUC_pkgs/call_m_hardware/call_m_drivers/

#export files for 'call_m_hardware/call_m_triorb_ros2' NUC version
cp -r src/call_m_hardware/call_m_triorb_ros2/ exports/call_m_NUC_pkgs/call_m_hardware/

#export files for 'call_m_start_all' NUC version
mkdir exports/call_m_NUC_pkgs/call_m_start_all/
mkdir exports/call_m_NUC_pkgs/call_m_start_all/config
mkdir exports/call_m_NUC_pkgs/call_m_start_all/launch
cp src/call_m_start_all/launch/call_m_start_NUC.launch.py exports/call_m_NUC_pkgs/call_m_start_all/launch/
cp src/call_m_start_all/CMakeLists.txt exports/call_m_NUC_pkgs/call_m_start_all/
cp src/call_m_start_all/package.xml exports/call_m_NUC_pkgs/call_m_start_all/

#export files for 'call_m_teleoperation' can be useful if needed to control robot by pluging a joystick directly into it
cp -r src/call_m_teleoperation/ exports/call_m_NUC_pkgs/

echo "-- NUC packages exported in /exports/call_m_NUC_pkgs/"

#JETSON packages
#export files for 'call_m_hardware/call_m_drivers' JETSON version
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/src
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/config
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/launch
cp -r src/call_m_hardware/call_m_drivers/src/ exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/
cp src/call_m_hardware/call_m_drivers/config/cam1_zedm.yaml exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/config/
cp src/call_m_hardware/call_m_drivers/config/cam2_zedm.yaml exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/config/
cp src/call_m_hardware/call_m_drivers/launch/bot_JETSON.launch.py exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/launch/
cp src/call_m_hardware/call_m_drivers/CMakeLists.txt exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/
cp src/call_m_hardware/call_m_drivers/package.xml exports/call_m_JETSON_pkgs/call_m_hardware/call_m_drivers/

#export files for 'call_m_start_all' JETSON version
mkdir exports/call_m_JETSON_pkgs/call_m_start_all/
mkdir exports/call_m_JETSON_pkgs/call_m_start_all/config
mkdir exports/call_m_JETSON_pkgs/call_m_start_all/launch
cp src/call_m_start_all/launch/call_m_start_JETSON.launch.py exports/call_m_JETSON_pkgs/call_m_start_all/launch/
cp src/call_m_start_all/CMakeLists.txt exports/call_m_JETSON_pkgs/call_m_start_all/
cp src/call_m_start_all/package.xml exports/call_m_JETSON_pkgs/call_m_start_all/

echo "-- JETSON packages exported in /exports/call_m_JETSON_pkgs/"
echo "Don't forget to install dependencies and 'included_external_packages' of workspace manually in host machine"

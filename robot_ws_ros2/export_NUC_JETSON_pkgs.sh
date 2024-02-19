rm -r exports/
mkdir exports/
touch exports/COLCON_IGNORE
mkdir exports/call_m_JETSON_pkgs/ exports/call_m_NUC_pkgs/

#NUC packages
#export files for 'call_m_hardware' NUC version
mkdir exports/call_m_NUC_pkgs/call_m_hardware/
mkdir exports/call_m_NUC_pkgs/call_m_hardware/src
mkdir exports/call_m_NUC_pkgs/call_m_hardware/config
mkdir exports/call_m_NUC_pkgs/call_m_hardware/launch
cp -r src/call_m_hardware/src/ exports/call_m_NUC_pkgs/call_m_hardware/
cp src/call_m_hardware/launch/bot_NUC.launch.py exports/call_m_NUC_pkgs/call_m_hardware/launch/
cp src/call_m_hardware/config/ekf.yaml exports/call_m_NUC_pkgs/call_m_hardware/config/
cp src/call_m_hardware/CMakeLists.txt exports/call_m_NUC_pkgs/call_m_hardware/
cp src/call_m_hardware/package.xml exports/call_m_NUC_pkgs/call_m_hardware/

#export files for 'call_m_start_all' NUC version
mkdir exports/call_m_NUC_pkgs/call_m_start_all/
mkdir exports/call_m_NUC_pkgs/call_m_start_all/config
mkdir exports/call_m_NUC_pkgs/call_m_start_all/launch
cp src/call_m_start_all/launch/call_m_start_NUC.launch.py exports/call_m_NUC_pkgs/call_m_start_all/launch/
cp src/call_m_start_all/CMakeLists.txt exports/call_m_NUC_pkgs/call_m_start_all/
cp src/call_m_start_all/package.xml exports/call_m_NUC_pkgs/call_m_start_all/

echo "-- NUC packages exported in /exports/call_m_NUC_pkgs/"

#JETSON packages
#export files for 'call_m_hardware' JETSON version
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/src
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/config
mkdir exports/call_m_JETSON_pkgs/call_m_hardware/launch
cp -r src/call_m_hardware/src/ exports/call_m_JETSON_pkgs/call_m_hardware/
cp src/call_m_hardware/config/cam1_zedm.yaml exports/call_m_JETSON_pkgs/call_m_hardware/config/
cp src/call_m_hardware/config/cam2_zedm.yaml exports/call_m_JETSON_pkgs/call_m_hardware/config/
cp src/call_m_hardware/launch/bot_JETSON.launch.py exports/call_m_JETSON_pkgs/call_m_hardware/launch/
cp src/call_m_hardware/CMakeLists.txt exports/call_m_JETSON_pkgs/call_m_hardware/
cp src/call_m_hardware/package.xml exports/call_m_JETSON_pkgs/call_m_hardware/

#export files for 'call_m_start_all' JETSON version
mkdir exports/call_m_JETSON_pkgs/call_m_start_all/
mkdir exports/call_m_JETSON_pkgs/call_m_start_all/config
mkdir exports/call_m_JETSON_pkgs/call_m_start_all/launch
cp src/call_m_start_all/launch/call_m_start_JETSON.launch.py exports/call_m_JETSON_pkgs/call_m_start_all/launch/
cp src/call_m_start_all/CMakeLists.txt exports/call_m_JETSON_pkgs/call_m_start_all/
cp src/call_m_start_all/package.xml exports/call_m_JETSON_pkgs/call_m_start_all/

echo "-- JETSON packages exported in /exports/call_m_JETSON_pkgs/"
echo "Don't forget to install dependencies in host machine"

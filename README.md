# Install ROS Jazzy and MAVROS
- https://github.com/IOES-Lab/ROS2_Jazzy_MacOS_Native_AppleSilicon
- Installs at ~/ros2_jazzy, activate with source ~/ros2_jazzy/activate_ros

# Install ROS-GZ
- https://github.com/IOES-Lab/ros_gz_for_mac
- Installs at ~/ros_gz_ws, activate with source ~/ros_gz_ws/install/setup.zsh

# Install MAVROS
- https://github.com/IOES-Lab/ROS2_MAVROS_AppleSilicon
- Installs at ~/mavros_ws, activate with source ~/mavros_ws/install/setup.zsh

# Set-up

```bash

# Install Ardupilot with MAVProxy
brew update
brew install genromfs
brew install gcc-arm-none-eabibrew install gawk
brew install gawk
pip install wxPython gnureadline billiard numpy pyparsing MAVProxy
cd
git clone --recurse-submodules git@github.com:IOES-Lab/ardupilot.git
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
cd $HOME/ardupilot
$HOME/ardupilot/modules/waf/waf-light configure --board=sitl
$HOME/ardupilot/modules/waf/waf-light build

# Install ArduPilot/ardupilot_gazebo
brew update
brew install rapidjson
brew install opencv gstreamer
cd
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# Create workspace
mkdir -p ~/ros_gz_rover/src && cd ~/ros_gz_rover/src

# clone ros-gz (ros2 jazzy branch)
git clone https://github.com/gazebosim/ros_gz.git -b jazzy

# clone this repo
git clone https://github.com/IOES-Lab/ros_gz_rover.git -b jazzy

# clone ArduPilot/SITL_Models
git clone https://github.com/ArduPilot/SITL_Models.git

# clone ArduPilot/ardupilot_gazebo
brew update
brew install rapidjson
brew install opencv gstreamer
git clone https://github.com/ArduPilot/ardupilot_gazebo

# Build
cd ~/ros_gz_rover-gz
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib -DCMAKE_CXX_STANDARD=17

# Run First terminal
# ROS명령어로 ros_gz를 통해 gazebo상에 Rover를 구현하고 gz 메시지를 모두 ros 메시지로 발신
# 발신된 ros 메시지를 rviz로 가시화
# gazebo상의 rover가 실제 하드웨어인척 작동하는 SITL(Simulation-In-The-Loop)상황
# gazebo상의 rover 주소는 udp://127.0.0.1:14550@14555 (하드웨어였다면 /usb/serial**인것)
source ~/ros2_jazzy/activate_ros
source ~/ros_gz_ws/install/setup.zsh
source ~/ros_gz_rover/install/setup.zsh
source ~/mavros_ws/install/setup.zsh
source ~/ros_gz_rover/install/setup.zsh
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ros_gz_rover/build/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ros_gz_rover/build/ardupilot_gazebo/models:$HOME/ros_gz_rover/build/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
ros2 launch ros_gz_rover rover.launch.py

# Run second terminal
# MAVROS를 통해 ROS메시지를 GCS와 통신할 수 있도록 mavlink프로토콜로 발신해주는 노드
# 입력값에 gazebo상의 rover 주소 udp://127.0.0.1:14550@14555를 입력
# 이전에 matek에서 imu값을 ros topic으로 볼때도 사용했던 그것
source ~/ros2_jazzy/activate_ros
source ~/ros_gz_ws/install/setup.zsh
source ~/ros_gz_rover/install/setup.zsh
source ~/mavros_ws/install/setup.zsh
ros2 launch ros_gz_rover mavros.launch.py

# Run third terminal
# 아두파일럿을 컨트롤 하기 위한 터미널 명령창
# 명령창을 구동하는것과 동시에 외부 GCS에서도 컨트롤 할 수 있도록 (proxy개념) 외부 GCS의 아이피주소 입력
# 여기서는 VMWare Fusion에서 구동되는 윈도우 머신의 Mission Planner를 사용하기 위해
# VMWare Fusion 윈도우 머신 아이피 주소를 입력
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
sim_vehicle.py -N -v Rover -f rover-skid --model JSON --mavproxy-args="--out=udp:192.168.0.136:14550"

# Run third terminal
# 맥에서는 gz sim 이 한번에 안켜지고 -s로 서버를 켜고 -g로 gui를 켜야 함
# 첫번째 터미널에서 -s로 서버가 돌아가는 중이니 -g로 gui만 켜는 명령어
gz sim -v4 -g
```
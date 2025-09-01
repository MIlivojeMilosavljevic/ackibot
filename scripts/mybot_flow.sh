#!/bin/bash

exit 0


##############################################################################
# Terminal 1

source /opt/ros/jazzy/setup.bash
export RCUTILS_COLORIZED_OUTPUT=1

# Build all.
colcon build --symlink-install
# Build specific package.
colcon build --symlink-install --packages-select mybot_node
# On RPi there is no gazebo.
colcon build --symlink-install --packages-ignore mybot_gazebo
# On RPi
colcon build --symlink-install --packages-ignore mybot_gazebo nav2_mppi_controller joy

source install/setup.sh

# Running ROS2 for robot
# In comment are default parameters that could be changed.
ros2 launch mybot_bringup robot.launch.py #en_imu:=false en_lidar:=false en_teleop:=false joypad:=ps3

# For agrobot from PC.
ros2 launch mybot_bringup robot.launch.py en_imu:=false en_lidar:=false joypad:=ps3

# Running ROS2 for Gazebo

ros2 launch mybot_gazebo empty_world.launch.py


##############################################################################
#Terminal 2: Debug stuff

# Check in RVIZ movement of the robot
ros2 launch mybot_bringup rviz2.launch.py

# joypad for gazebo
ros2 launch teleop_twist_joy teleop-launch.py \
    config_filepath:=`ros2 pkg prefix --share  mybot_teleop`/config/sony.config.yaml

# Check the look of URDF.
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share mybot_description`/urdf/mybot.urdf


##############################################################################
#Terminal 3

# Plugins -> Topics -> Topic Monitor
ros2 run rqt_gui rqt_gui

# Debugging Arduino.
picocom -b 115200 /dev/ttyUSB1

# Plot
ros2 launch mybot_slam_nav plot.launch.py

# Just node.
ros2 run mybot_node fw_node -i /dev/ttyUSB0 --ros-args --params-file `ros2 pkg prefix --share mybot_bringup`/param/mybot.yaml -r cmd_vel:=/cmd_vel_node

# Command line controlling.
function drive_turn() {
    ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear:  {x: $1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $2}}"
}

function drive_turn() {
    ros2 topic pub -1 /cmd_vel_node geometry_msgs/msg/TwistStamped "
header:
  stamp: now
  frame_id: nav
twist:
  linear:
    x: $1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: $2
"
}

function drive_turn() {
    ros2 topic pub --rate 10 /cmd_vel_node geometry_msgs/msg/TwistStamped "
header:
  stamp: now
  frame_id: nav
twist:
  linear:
    x: $1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: $2
"
}

# Forward
drive_turn 1.0 0
# Backward
drive_turn -1.0 0
# Left
drive_turn 0 1.0
# Right
drive_turn 0 -1.0
# Stop
drive_turn 0 0




# Debugging topics.
ros2 node info /teleop_twist_joy_node
ros2 topic echo /joy
ros2 topic echo /cmd_vel

# For RVIZ
ros2 topic echo /joint_states
ros2 topic echo /odom
ros2 topic echo /tf


# TODO config
ros2 service type /teleop_twist_joy_node/list_parameters
ros2 service call /teleop_twist_joy_node/list_parameters rcl_interfaces/srv/ListParameters

#TODO Try
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo turtlebot3
ros2 run tf2_ros tf2_monitor world turtlebot3

##############################################################################

# LIDAR.
# need LIDAR to be on /dev/ttyUSB0
RCL_LOG_LEVEL=debug ros2 launch rplidar_ros rplidar.launch.py
ros2 launch rplidar_ros view_rplidar.launch.py
# A1, A2
ros2 launch mybot_bringup lidar.launch.py serial_baudrate:=115200 serial_port:=/dev/ttyUSB2

ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB2 --log-level debug

# C1
pushd ~/mybot/mybot_ws/
pushd src/
git clone https://github.com/Slamtec/rplidar_ros/ -b ros2
popd
colcon build --symlink-install --packages-select rplidar_ros
source install/setup.sh
ros2 pkg prefix --share rplidar_ros
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB1
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB1 -p serial_baudrate:=460800 --log-level debug


##############################################################################
# IMU

# SBC:
# VCC 2 5V
# GND 6
# SDA 3
# SCL 5
i2cdetect -y -r 1

ros2 run imu_bno055 bno055_i2c_node

# PC:
# Over UART
ros2 launch bno055 bno055.launch.py

# Plugins -> Topics -> Topic Monitor
ros2 run rqt_gui rqt_gui

ros2 run plotjuggler plotjuggler

##############################################################################

ros2 launch mybot_slam_nav teleop.launch.py prefix:='sbc' joypad:='ps3'
ros2 launch teleop_twist_joy teleop-launch.py \
    config_filepath:=`ros2 pkg prefix --share mybot_bringup`/config/ps3.config.yaml


# Testing twist_mux idle_expire
ros2 topic pub \
    -r 5 \
    /cmd_vel_nav_stamped\
    geometry_msgs/msg/TwistStamped \
"
header:
  stamp: now
  frame_id: nav
twist:
  linear:
    x: 0.1
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
"

##############################################################################
# Bluetooth joypad

sudo systemctl enable bluetooth

rfkill list
# must be no at soft and hard blocked
rfkill unblock bluetooth

bluetoothctl power on
#agent on
#default-agent
#pairable on
#discoverable on
bluetoothctl scan on
bluetoothctl devices


mybot_pair_bluetooth_joypad.py -v

ls /dev/input/js*
jstest /dev/input/js0
 
# joy probem
ros2 run joy joy_node
ros2 topic echo /joy

ros2 run joy joy_node --ros-args --log-level debug

ros2 run mybot_teleop joy_node --ros-args --log-level debug

######

# Obsolete.
sudo pip install --break-system-packages ds4drv
sudo sed -i 's/SafeConfigParser/ConfigParser/g' /usr/local/lib/python3.12/dist-packages/ds4drv/config.py

ds4drv --hidraw
sudo rmmod hid_playstation
echo "blacklist hid_playstation" | sudo tee -a /etc/modprobe.d/blacklist.conf
#lsmod | grep uinput

######

push src
# 3.3.0-3noble.20250624.004034
git clone https://github.com/ros-drivers/joystick_drivers
pushd joystick_drivers
git checkout tags/3.3.0
mv joy ../
popd
rm -rf joystick_drivers
popd

colcon build --symlink-install --packages-select joy
source install/setup.sh
ros2 pkg prefix --share joy
ros2 run joy joy_node

######

git clone -b release-2.30.x --depth 1 https://github.com/libsdl-org/SDL.git

cd ~/mybot/SW/joy_SDL_test/

sudo apt install libsdl2-tests

/usr/libexec/installed-tests/SDL2/testjoystick

##############################################################################


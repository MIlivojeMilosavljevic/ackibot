#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Install usual ROS2 stuff.
source $SCRIPT_DIR/install_ros2.sh

# More user-friendly.
echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> ~/.profile

# Remote connection.
echo 'export ROS_DOMAIN_ID=30' >> ~/.profile

# Automation, but dirty env.
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.profile
echo "source $SCRIPT_DIR/../install/setup.sh" >> ~/.profile
echo "export PATH=\$PATH:$SCRIPT_DIR/" >> ~/.profile

sudo apt -y install \
    libserial-dev \
    python3-pyudev \
    picocom \
    sshpass

sudo usermod -a -G dialout $USER
sudo usermod -a -G i2c $USER


# Bluetooth stuff
sudo apt -y install jstest-gtk bluez-tools
sudo usermod -a -G bluetooth $USER
echo joydev | sudo tee -a /etc/modules
sudo sed -i 's/#ControllerMode = dual/ControllerMode = dual/g' /etc/bluetooth/main.conf
sudo systemctl disable bluetooth
sudo systemctl enable bluetooth
sudo rfkill unblock bluetooth
sudo bluetoothctl power on
sudo bluetoothctl scan on
sudo bluetoothctl devices





sudo apt -y install ros-$ROS_DISTRO-urdf-tutorial

# Need on SBC
sudo apt -y install \
    ros-$ROS_DISTRO-rplidar-ros \
    ros-$ROS_DISTRO-twist-mux \
    ros-$ROS_DISTRO-twist-stamper
    
sudo apt -y install \
    ros-$ROS_DISTRO-bno055 \
    ros-$ROS_DISTRO-robot-localization




exit 0

################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(ackibot_utils)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

################################################################################
# Build
################################################################################


#############
## Install ##
#############

# Install Python modules
ament_python_install_package(ackibot_utils)


ament_package()
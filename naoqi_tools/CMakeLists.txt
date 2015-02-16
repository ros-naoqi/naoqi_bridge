cmake_minimum_required(VERSION 2.8.3)
project(naoqi_tools)

# We need to find catkin for the rest of the script to work
find_package(catkin REQUIRED)# COMPONENTS
#  roscpp
#  tf
#  message_filters
#  sensor_msgs
#)

# Nothing that can be imported by other modules
catkin_package()

catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/generate_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

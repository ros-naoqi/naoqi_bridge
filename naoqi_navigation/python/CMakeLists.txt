cmake_minimum_required(VERSION 2.8.3)
project(naoqi_navigation)

set( MESSAGE_DEPENDENCIES
    geometry_msgs
    octomap_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ${MESSAGE_DEPENDENCIES}
)

catkin_package()
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# install launch files
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   FILES_MATCHING PATTERN "*.launch"
)

catkin_install_python(PROGRAMS
  nodes/pod.py nodes/localization.py nodes/exploration.py nodes/loading_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

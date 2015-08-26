cmake_minimum_required(VERSION 2.8.3)
project(naoqi_driver_py)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
)

# This will install python libraries
catkin_python_setup()

 # auto-generate dynamic reconfiguration GUI before catkin_package()
generate_dynamic_reconfigure_options(cfg/NaoqiSpeech.cfg)

# Information for other packages to import this module
catkin_package(
  CFG_EXTRAS naoqi_driver-extras.cmake
)

#And this will install python nodes
catkin_install_python(PROGRAMS nodes/naoqi_joint_states.py nodes/naoqi_logger.py nodes/naoqi_moveto.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Install cmake file
install(FILES cmake/FindNAOqi.cmake
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/
)

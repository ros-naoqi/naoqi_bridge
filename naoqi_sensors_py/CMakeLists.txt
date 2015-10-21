cmake_minimum_required(VERSION 2.8)
project(naoqi_sensors_py)

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  dynamic_reconfigure
  octomap_msgs
)

# deal with the Python code
catkin_python_setup()

# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread program_options)

# auto-generate dynamic reconfiguration GUI before catkin_package()
generate_dynamic_reconfigure_options(cfg/NaoqiCamera.cfg
                                     cfg/NaoqiMicrophone.cfg)

catkin_package()

add_subdirectory(./src)

# install sensor scripts
catkin_install_python(PROGRAMS nodes/camera.py nodes/microphone.py nodes/octomap.py nodes/sonar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install launch files
install(FILES launch/camera.launch 
              launch/microphone.launch
              launch/octomap.launch
              launch/sonar.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# install camera files
install(DIRECTORY share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

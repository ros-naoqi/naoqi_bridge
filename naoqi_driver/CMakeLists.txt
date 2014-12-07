cmake_minimum_required(VERSION 2.8.3)
project(naoqi_driver)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    sensor_msgs
    geometry_msgs
    naoqi_msgs
    dynamic_reconfigure
    diagnostic_aggregator
    diagnostic_msgs)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue
)

find_package(Boost QUIET COMPONENTS program_options signals)

# This will install python libraries
catkin_python_setup()

 # auto-generate dynamic reconfiguration GUI before catkin_package()
generate_dynamic_reconfigure_options(cfg/NaoqiSpeech.cfg)

if( ${NAOqi_FOUND} AND ${Boost_FOUND} )
  message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")

  # Information for other packages to import this module
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES naoqi_node
    CFG_EXTRAS naoqi_driver-extras.cmake
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${NAOqi_INCLUDE_DIRS}
    )

  add_library(naoqi_node src/naoqi_node.cpp)
  target_link_libraries(naoqi_node
                        ${catkin_LIBRARIES}
                        ${NAOqi_LIBRARIES}
                        ${Boost_LIBRARIES}
                        )

  add_executable(naoqi_joint_states_cpp src/naoqi_joint_states.cpp)
  target_link_libraries(naoqi_joint_states_cpp
                        naoqi_node
                       ${catkin_LIBRARIES}
                       ${NAOqi_LIBRARIES}
                       ${Boost_LIBRARIES}
                       )
  add_dependencies(naoqi_joint_states_cpp ${catkin_EXPORTED_TARGETS})

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  install(TARGETS naoqi_node
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          )
  install(TARGETS naoqi_joint_states_cpp
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
else()
  message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
  catkin_package(CFG_EXTRAS naoqi_driver-extras.cmake)
endif()

# add a diagnostic analyzer plugin
add_library(naoqi_diagnostic_analyzers src/naoqi_joints_analyzer.cpp)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(naoqi_diagnostic_analyzers ${catkin_LIBRARIES})
install(TARGETS naoqi_diagnostic_analyzers DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES naoqi_analyzer_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#And this will install python nodes
install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Install cmake file
install(FILES cmake/FindNAOqi.cmake
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/
)

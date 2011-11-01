cmake_minimum_required(VERSION 2.8)
project(nao_camera)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  driver_base
  dynamic_reconfigure
  image_transport
#  nodelet
  roscpp
  sensor_msgs
  tf)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue)


# This driver uses Boost threads
find_package(Boost REQUIRED COMPONENTS thread program_options)

# auto-generate dynamic reconfiguration GUI before catkin_package()
generate_dynamic_reconfigure_options(cfg/NaoCamera.cfg)

catkin_package(CATKIN_DEPENDS
                 camera_info_manager
                 diagnostic_updater
                 dynamic_reconfigure
                 image_transport
 #                nodelet
                 roscpp
                 tf
               DEPENDS Boost)
			   #LIBRARIES src/nodes/camera1394_nodelet)

if( ${NAOqi_FOUND} AND ${Boost_FOUND} )
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS})

    add_executable(cameras src/nao_camera.cpp src/nao_camera_node.cpp src/nao_node.cpp)
    target_link_libraries(cameras
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${NAOqi_LIBRARIES})
    add_dependencies(cameras ${catkin_EXPORTED_TARGETS})

    install(TARGETS cameras
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

	## install pluginlib definition for the nodelet
	#install(FILES camera1394_nodelet.xml
	#	    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
    message(FATAL_ERROR "Cannot find NAOqi C++ SDK! Set NAOqi_DIR to the SDK prefix.")
endif()



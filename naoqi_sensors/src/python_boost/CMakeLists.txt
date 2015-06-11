find_package(octomap)
find_package(Boost COMPONENTS python)
set(Python_ADDITIONAL_VERSIONS 2.7)
find_package(PythonLibs REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_PATH} ${Boost_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})

# add the boost Python module
add_library(octomap_python octomap_python.cpp)
target_link_libraries(octomap_python ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
set_target_properties(octomap_python PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost
                      OUTPUT_NAME octomap_python
                      PREFIX ""
)

install(TARGETS octomap_python
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost
)

# add an __init__.py file to be able to import the file
configure_file(__init__.py.plain.in
               ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost/__init__.py
               @ONLY
)

install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost/__init__.py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost
)

cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED owd_msgs roscpp roslib openrave_catkin)
catkin_package()

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

include_directories(
    "include/${PROJECT_NAME}"
    ${OpenRAVE_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${OpenRAVE_LIBRARY_DIRS}
    ${YamlCpp_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)
add_definitions(-DNO_MAC_TRAJECTORY)

openrave_plugin("${PROJECT_NAME}_plugin"
    src/BHController.cpp
    src/BHTactileSensor.cpp
    src/BarrettFTSensor.cpp
    src/HandstateSensor.cpp
    src/OWDController.cpp
    src/AllOWDPlugins.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
    ${YamlCpp_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies("${PROJECT_NAME}_plugin"
    ${catkin_EXPORTED_TARGETS}
)

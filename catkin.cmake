cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED owd_msgs roscpp)
catkin_package(
    DEPENDS openrave owd_msgs
)

find_package(OpenRAVE REQUIRED)
set(OpenRAVE_LIBRARY_SUFFIX "0.9")

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

include_directories(
    include/${PROJECT_NAME}
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

add_library(${PROJECT_NAME}_plugin
    src/BHController.cpp
    src/BHTactileSensor.cpp
    src/OWDController.cpp
    src/AllOWDPlugins.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
    ${OpenRAVE_LIBRARIES}
    ${YamlCpp_LIBRARIES}
    ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)
add_dependencies(${PROJECT_NAME}_plugin
    ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}_plugin
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)

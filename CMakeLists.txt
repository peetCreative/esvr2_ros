cmake_minimum_required(VERSION 2.8.3)
project(esvr2_ros)

add_compile_options(-std=c++14)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake")
SET(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
    mediassist3_panda_pivoting
    cv_bridge
    roscpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    message_filters
    pluginlib
)

#find_package(Esvr2)
SET(Esvr2_HOME /path/to/esvr2/install/)
SET(Esvr2_LIBRARIES ${Esvr2_HOME}/lib/libEsvr2_lib.so)
SET(Esvr2_INCLUDE_DIR ${Esvr2_HOME}/include/esvr2)
SET(Esvr2_RESOURCE_FILE ${Esvr2_HOME}/bin/Resources.cfg)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES esvr2_ros
    CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )

add_definitions(-DPLUGIN_FOLDER="${CMAKE_CURRENT_BIN_DIR}/${CMAKE_BUILD_TYPE}/" )
add_definitions(-DRESOURCE_FOLDER="${CMAKE_CURRENT_BIN_DIR}/Data/")
add_definitions(-DUSE_ROS)

message(Esvr2_INCLUDE_DIR:${Esvr2_INCLUDE_DIR})

include_directories(
    include/esvr2_ros
    ${catkin_INCLUDE_DIRS}
    ${Esvr2_INCLUDE_DIR}
)

macro( add_recursive dir retVal )
    file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

add_recursive(src SOURCES)

add_executable( ${PROJECT_NAME}
        ${SOURCES} )
target_link_libraries( ${PROJECT_NAME}
        ${catkin_LIBRARIES} ${Esvr2_LIBRARIES})

cmake_minimum_required(VERSION 3.16)
project(esvr2_ros)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake")
SET(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    std_srvs
    sensor_msgs
    image_transport
    geometry_msgs
    tf2
    tf2_ros
    message_filters
    pluginlib
    pivot_control_messages_ros
)

#find_package(Esvr2)
SET(Esvr2_HOME)
SET(Esvr2_LIBRARIES ${Esvr2_HOME}/lib/libEsvr2.so)
SET(Esvr2_INCLUDE_DIR ${Esvr2_HOME}/include/esvr2)
SET(Esvr2_RESOURCE_FILE ${Esvr2_HOME}/bin/Resources.cfg)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES esvr2_ros
    CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )

if (TARGET pivot_control_messages)
    message("found pivot_control_messages")
else()
    add_subdirectory(pivot_control_messages)
endif()

include_directories(
    include/esvr2_ros
    ${catkin_INCLUDE_DIRS}
    ${Esvr2_INCLUDE_DIR}
    pivot_control_messages/include
)

macro( add_recursive dir retVal )
    file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

add_recursive(src/screen_player SOURCES_SCREEN_PLAYER)

add_executable( screen_player
        ${SOURCES_SCREEN_PLAYER})
add_dependencies( screen_player
        ${catkin_EXPORTED_TARGETS})

target_link_libraries( screen_player
        ${catkin_LIBRARIES})

add_recursive(src/esvr2_ros SOURCES_ESVR2_ROS)
add_executable( ${PROJECT_NAME}
        ${SOURCES_ESVR2_ROS})

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        pivot_control_messages_ros_gencpp)

target_compile_definitions( ${PROJECT_NAME} PRIVATE
        "RESOURCES_FILE=\"${Esvr2_RESOURCE_FILE}\"" )
target_link_libraries( ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Esvr2_LIBRARIES}
        pivot_control_messages
        pivot_control_messages_ros)

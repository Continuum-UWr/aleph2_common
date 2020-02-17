cmake_minimum_required(VERSION 2.8.3)
project(aleph2_manip)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    pluginlib
    kdl_parser
    kdl_conversions)

catkin_package(
    CATKIN_DEPENDS 
        roscpp 
        moveit_core 
        pluginlib 
        geometry_msgs 
        sensor_msgs 
        geometry_msgs 
        moveit_msgs
        kdl_parser
        kdl_conversions
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS})

add_library(aleph2_manip_kinematics
    src/aleph2_manip_kinematics)

add_executable(test_ik
    src/test_ik.cpp)

target_link_libraries(test_ik
    ${catkin_LIBRARIES})

add_executable(test_ik1
    nodes/test_ik1.cpp)

target_link_libraries(test_ik1
    aleph2_manip_kinematics
    ${catkin_LIBRARIES})
cmake_minimum_required(VERSION 2.8.3)
project(aleph2_manip)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_core
    pluginlib
    kdl_parser
    kdl_conversions
    message_generation  
    tf2_ros
    tf2_geometry_msgs
)

add_message_files(
  FILES
  KinematicsCommand.msg
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS 
        roscpp 
        moveit_core 
        pluginlib 
        geometry_msgs 
        sensor_msgs 
        geometry_msgs 
        moveit_msgs
        std_srvs
        kdl_parser
        kdl_conversions
        message_runtime
        tf2_ros
        tf2_geometry_msgs
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS})

add_library(aleph2_manip_kinematics
    src/aleph2_manip_kinematics)

add_executable(manip_kinematics_control_node
    nodes/manip_kinematics_control_node.cpp)

target_link_libraries(manip_kinematics_control_node
    aleph2_manip_kinematics
    ${catkin_LIBRARIES})
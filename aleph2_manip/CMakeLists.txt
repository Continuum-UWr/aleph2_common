cmake_minimum_required(VERSION 2.8.3)
project(aleph2_manip)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation  
    moveit_core
    pluginlib
    geometry_msgs 
    sensor_msgs 
    moveit_msgs
    kdl_parser
    kdl_conversions
    tf2_ros
    tf2_geometry_msgs
    dynamic_reconfigure
)

add_message_files(
  FILES
  KinematicsCommand.msg
)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/Kinematics.cfg
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
        roscpp 
        message_runtime
        moveit_core 
        pluginlib 
        geometry_msgs 
        sensor_msgs 
        moveit_msgs
        kdl_parser
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS})

add_library(aleph2_manip_kinematics
    src/aleph2_manip_kinematics)

add_executable(manip_kinematics_control_node
    nodes/manip_kinematics_control_node.cpp)

add_dependencies(manip_kinematics_control_node
    ${PROJECT_NAME}_generate_messages_cpp
    ${PROJECT_NAME}_gencfg)

target_link_libraries(manip_kinematics_control_node
    aleph2_manip_kinematics
    ${catkin_LIBRARIES})
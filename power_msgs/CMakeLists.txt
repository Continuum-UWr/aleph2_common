cmake_minimum_required(VERSION 2.8.3)
project(power_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  PowerStatus.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
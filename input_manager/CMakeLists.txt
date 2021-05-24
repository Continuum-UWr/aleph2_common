cmake_minimum_required(VERSION 3.0.2)
project(input_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(SDL2 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
    Input.msg
    DeviceList.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
  DEPENDS SDL2
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/rostalker.cpp
  src/joystick.cpp
  src/main.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SDL2_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

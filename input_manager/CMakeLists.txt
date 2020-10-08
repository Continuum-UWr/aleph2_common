cmake_minimum_required(VERSION 2.8.3)
project(input_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  input_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs input_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(input_manager 
  src/rostalker.cpp
  src/joystick.cpp
  src/main.cpp
)

add_dependencies(input_manager 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(input_manager
  SDL2
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS input_manager input_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

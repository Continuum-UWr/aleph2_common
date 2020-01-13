cmake_minimum_required(VERSION 2.8.3)
project(aleph2_description)

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY
  meshes
  rviz
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

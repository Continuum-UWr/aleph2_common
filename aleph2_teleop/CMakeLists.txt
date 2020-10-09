cmake_minimum_required(VERSION 2.8.3)
project(aleph2_teleop)

find_package(catkin)

catkin_package()

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  PROGRAMS scripts/key_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
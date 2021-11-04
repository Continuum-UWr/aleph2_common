cmake_minimum_required(VERSION 3.5)
project(aleph2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    textures
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

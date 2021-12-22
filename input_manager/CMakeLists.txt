cmake_minimum_required(VERSION 3.8)
project(input_manager)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SDL2 REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "msg/DeviceList.msg"
  "msg/Input.msg"
  LIBRARY_NAME ${PROJECT_NAME}
)

add_executable(input_manager
  src/main.cpp
  src/joystick.cpp
  src/rostalker.cpp
)

include_directories(${YAML_CPP_INCLUDE_DIRS})

target_link_libraries(input_manager
  -ludev
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(input_manager
  rclcpp
  SDL2
  yaml-cpp
)

rosidl_target_interfaces(input_manager
  input_manager_interfaces "rosidl_typesupport_cpp"
)

install(TARGETS
  input_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

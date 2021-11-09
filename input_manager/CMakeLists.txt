cmake_minimum_required(VERSION 3.8)
project(input_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SDL2 REQUIRED)

rosidl_generate_interfaces(input_manager_interfaces
  "msg/DeviceList.msg"
  "msg/Input.msg"
)

add_executable(input_manager
  src/main.cpp
  src/joystick.cpp
  src/rostalker.cpp
)

ament_target_dependencies(input_manager
  rclcpp
  SDL2
)

rosidl_target_interfaces(input_manager
  input_manager_interfaces "rosidl_typesupport_cpp"
)

install(TARGETS
  input_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

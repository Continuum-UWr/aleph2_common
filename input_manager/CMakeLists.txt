cmake_minimum_required(VERSION 3.8)
project(input_manager)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SDL2 REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  "msg/DeviceInfo.msg"
  "msg/DeviceList.msg"
  "msg/Input.msg"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp"
)

add_executable(input_manager
  src/main.cpp
  src/joystick.cpp
  src/rostalker.cpp
)
target_include_directories(input_manager
  PRIVATE 
    ${YAML_CPP_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)
target_link_libraries(input_manager
  -ludev
  ${YAML_CPP_LIBRARIES}
  SDL2
  rclcpp::rclcpp
  ${cpp_typesupport_target}
)

install(TARGETS
  input_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

{ lib, buildRosPackage,cmake, ament-cmake, rosidl-default-generators, rclcpp, SDL2, yaml-cpp, libudev-zero}:
buildRosPackage {
  pname = "input_manager";
  version = "1.0";

  src = ./.;

  buildType = "ament_cmake";
  propagatedBuildInputs = [  ament-cmake rosidl-default-generators rclcpp SDL2 yaml-cpp libudev-zero];
  nativeBuildInputs = [ cmake ];
}
{ buildRosPackage, ament-cmake, rosidl-default-generators
, rosidl-default-runtime, ament-index-cpp, rclcpp, SDL2, yaml-cpp, libudev-zero
, ament-lint-auto, ament-cmake-cpplint, ament-cmake-lint-cmake
, ament-cmake-uncrustify }:
buildRosPackage {
  pname = "input_manager";
  version = "0.0.0";

  src = ./.;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake rosidl-default-generators ];
  propagatedBuildInputs = [
    rosidl-default-runtime
    ament-index-cpp
    rclcpp
    SDL2
    yaml-cpp
    libudev-zero
  ];
  checkInputs = [
    ament-lint-auto
    ament-cmake-cpplint
    ament-cmake-lint-cmake
    ament-cmake-uncrustify
  ];
  nativeBuildInputs = [ ament-cmake ];
}

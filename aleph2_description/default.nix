{ buildRosPackage, ament-cmake, xacro, robot-state-publisher, ament-lint-auto
, ament-cmake-lint-cmake, ament-cmake-xmllint }:
buildRosPackage {
  pname = "aleph2_description";
  version = "1.0";

  src = ./.;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [ xacro robot-state-publisher ];
  check-inputs = [ ament-lint-auto ament-cmake-lint-cmake ament-cmake-xmllint ];
  nativeBuildInputs = [ ament-cmake ];
}

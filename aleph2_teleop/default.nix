{ buildRosPackage, ament-cmake, joy, teleop-twist-joy, teleop-twist-keyboard
, ament-lint-auto, ament-cmake-lint-cmake, ament-cmake-xmllint }:
buildRosPackage {
  pname = "aleph2_teleop";
  version = "1.0";

  src = ./.;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [ joy teleop-twist-joy teleop-twist-keyboard ];
  check-inputs = [ ament-lint-auto ament-cmake-lint-cmake ament-cmake-xmllint ];
  nativeBuildInputs = [ ament-cmake ];
}

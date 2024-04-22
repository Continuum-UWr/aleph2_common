{ buildRosPackage, ament-cmake, joy, teleop-twist-joy, teleop-twist-keyboard
, ament-lint-auto, ament-cmake-lint-cmake, ament-cmake-xmllint }:
buildRosPackage {
  pname = "aleph2_teleop";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [ joy teleop-twist-joy teleop-twist-keyboard ];
  checkInputs = [ ament-lint-auto ament-cmake-lint-cmake ament-cmake-xmllint ];
  nativeBuildInputs = [ ament-cmake ];
}

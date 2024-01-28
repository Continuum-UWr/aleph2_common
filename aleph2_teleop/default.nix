{ lib, buildRosPackage,cmake, ament-cmake}:
buildRosPackage {
  pname = "aleph2_teleop";
  version = "1.0";

  src = ./.;

  buildType = "ament_cmake";
  propagatedBuildInputs = [  ament-cmake ];
  nativeBuildInputs = [ cmake ];
}
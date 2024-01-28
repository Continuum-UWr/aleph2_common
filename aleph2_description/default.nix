{ lib, buildRosPackage,cmake, ament-cmake}:
buildRosPackage {
  pname = "aleph2_description";
  version = "1.0";

  src = ./.;

  buildType = "ament_cmake";
  propagatedBuildInputs = [  ament-cmake ];
  nativeBuildInputs = [ cmake ];
}
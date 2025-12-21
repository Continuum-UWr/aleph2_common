{
  buildRosPackage,
  ament-cmake,
  xacro,
  robot-state-publisher,
  ament-lint-auto,
  ament-cmake-lint-cmake,
  ament-cmake-xmllint,
}:
buildRosPackage {
  pname = "aleph2-description";
  version = "0.0.0";

  src = ./.;

  doCheck = true;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    xacro
    robot-state-publisher
  ];
  checkInputs = [
    ament-lint-auto
    ament-cmake-lint-cmake
    ament-cmake-xmllint
  ];
  nativeBuildInputs = [ ament-cmake ];
}

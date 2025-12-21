{
  inputs = {
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    flake-utils.follows = "nix-ros-overlay/flake-utils";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
  };
  outputs = { self, nixpkgs, flake-utils, nix-ros-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = (import nixpkgs {
          system = system;
          overlays = [ nix-ros-overlay.overlays.default ];
        }).pkgs;
        ros = pkgs.rosPackages.jazzy;

        aleph2-description = ros.callPackage (import ./aleph2_description) { };
        aleph2-teleop = ros.callPackage (import ./aleph2_teleop) { };
        input-manager = ros.callPackage (import ./input_manager) { };

        devEnv = ros.buildEnv {
          paths =
            [ ros.ros-core aleph2-description aleph2-teleop input-manager ];
        };

      in {
        packages = {
          inherit aleph2-description aleph2-teleop input-manager devEnv;
          default = input-manager;
        };

        devShells.default = pkgs.mkShell { nativeBuildInputs = [ devEnv ]; };
        formatter = pkgs.nixfmt;
      });
}

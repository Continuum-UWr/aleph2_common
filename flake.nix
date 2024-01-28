{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs?ref=173b74db07f26344f3517716edd4bff6987b512d";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros-overlay = {
      url = "github:lopsided98/nix-ros-overlay?ref=42cae93f39f0d26134a884a1ec20d1e36dd14f66";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };

  };
  outputs = { self, nixpkgs, flake-utils, nix-ros-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = (import nixpkgs { system = system; overlays = [ ]; }).pkgs;
        ros = (import nixpkgs { system = system; overlays = [ nix-ros-overlay.overlays.default ]; }).pkgs.rosPackages.humble;

        aleph2_description = ros.callPackage (import ./aleph2_description) { };
        aleph2_teleop = ros.callPackage (import ./aleph2_teleop) { };
        input_manager = ros.callPackage (import ./input_manager) { };

      in
      {
        packages = {
          inherit aleph2_description aleph2_teleop input_manager;
          default = input_manager;

        };
      }
    );
}


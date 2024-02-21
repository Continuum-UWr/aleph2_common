{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    nix-ros-overlay = {
      url = "github:lopsided98/nix-ros-overlay";
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
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [

          ];
          inputsFrom = [ ];
          packages = [ ];
        };
      }
    );
}


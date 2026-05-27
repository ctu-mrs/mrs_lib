{
  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    devenv.url = "github:cachix/devenv";

    nixpkgs.follows = "devenv/nixpkgs";

    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    ros-nixpkgs.follows = "nix-ros-overlay/nixpkgs";

    mrs_cmake_repo.url = "github:ctu-mrs/mrs_cmake/nix";
    mrs_cmake_repo.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    mrs_cmake_repo.inputs.nix-ros-overlay.follows = "nix-ros-overlay";

    mrs_msgs_repo.url = "github:ctu-mrs/mrs_msgs/nix";
    mrs_msgs_repo.inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    mrs_msgs_repo.inputs.nix-ros-overlay.follows = "nix-ros-overlay";
  };

  outputs = inputs@{ flake-parts, ... }:

    flake-parts.lib.mkFlake { inherit inputs; } {

      # 1. Import the devenv module natively
      imports = [
        inputs.devenv.flakeModule
      ];

      systems = [ "x86_64-linux" ];

      # 3. Everything in here is automatically generated for each system above
      perSystem = { config, self', inputs', pkgs, system, ... }:

        let
          # Apply your ROS overlay for this specific system
          rosPkgs = import inputs.ros-nixpkgs {
            inherit system;
            overlays = [ inputs.nix-ros-overlay.overlays.default ];
          };

          ros = rosPkgs.rosPackages.jazzy;

          mrs_cmake = inputs.mrs_cmake_repo.packages.${system}.default;
          mrs_msgs = inputs.mrs_msgs_repo.packages.${system}.default;

          rosDeps = [
            ros.ros-core
            ros.ament-cmake-core
            ros.builtin-interfaces
            ros.sensor-msgs
            ros.std-srvs
            ros.std-msgs
            ros.nav-msgs
            ros.geometry-msgs
            ros.python-cmake-module
            ros.rosidl-default-runtime
            ros.tf2
            ros.tf2-geometry-msgs
            ros.tf2-eigen
            ros.visualization-msgs
            pkgs.eigen
            pkgs.yaml-cpp
            pkgs.boost
            mrs_cmake
            mrs_msgs
          ];
        in
        {
          # --- The Local Developer Environment ---
          # devenv.shells handles all the mkShell boilerplate behind the scenes
          devenv.shells.default = {

            name = "mrs_lib-dev-shell";

            _module.args = {
              inherit rosPkgs; # This passes the rosPkgs you defined above
              inherit rosDeps;
            };

            devenv.root =
              let
                folder = builtins.getEnv "PWD";
                isInsideWorkTree = folder != "";
              in
                if isInsideWorkTree
                then folder
                else "${./.}";

            imports = [ ./devenv.nix ];
          };

          # --- The C++ Package Builder ---
          packages.default = ros.buildRosPackage {
            pname = "mrs_lib";
            version = "2.0.0";
            src = ./.;
            buildType = "ament_cmake";
            nativeBuildInputs = [ ros.ament-cmake ros.rosidl-default-generators ];
            propagatedBuildInputs = rosDeps;
          };
        };

      # 4. Global flake configurations live at the bottom
      flake = {
        nixConfig = {
          extra-substituters = [ "https://ctu-mrs.cachix.org" "https://ros.cachix.org" "https://devenv.cachix.org" ];
          extra-trusted-public-keys = [ "ctu-mrs.cachix.org-1:dnw2ixFgGHfTb4bE1MWQTetAUJe9zqKUOBlrTjDuDMI=" "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" "devenv.cachix.org-1:w1cLUi8dv3hnoSPGAuibQv+f9TZLr6cv/Hm9XgU50cw=" ];
        };
      };
    };
}

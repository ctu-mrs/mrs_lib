{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    # This automatically loops through x86_64-linux, aarch64-linux, etc.
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ros = pkgs.rosPackages.jazzy;
      in {

        # We drop ${system} here because eachDefaultSystem handles it
        packages.default = ros.buildRosPackage {
          pname = "mrs_lib";
          version = "2.0.0";
          
          # Use path syntax, not string syntax
          src = ./.;
          
          buildType = "ament_cmake";
          
          nativeBuildInputs = [ 
            ros.ament-cmake 
            ros.rosidl-default-generators 
          ];
          
          buildInputs = [ 
            ros.ros-core
            ros.ament-cmake-core
            ros.builtin-interfaces
            ros.sensor-msgs
            ros.std-srvs
            ros.std-msgs
            ros.geometry-msgs
            ros.python-cmake-module
            # Added runtime requirement for messages
            ros.rosidl-default-runtime
          ];
        };

        devShells.default = pkgs.mkShell {
          name = "mrs_msgs";
          packages = [
            pkgs.colcon
            (ros.buildEnv {
              paths = [
                ros.ros-core
                ros.ament-cmake 
                ros.ament-cmake-core
                ros.builtin-interfaces
                ros.mrs-msgs
                ros.geometry-msgs
                ros.python-cmake-module
                # Required to run colcon build locally for messages
                ros.rosidl-default-generators
                ros.rosidl-default-runtime
              ];
            })
          ];
        };
      });

  # This configures Nix to download pre-built ROS binaries instead of compiling C++ from scratch
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}

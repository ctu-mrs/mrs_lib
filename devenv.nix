{ rosPkgs, rosDeps, ... }:
let
  ros = rosPkgs.rosPackages.jazzy;
  deps = rosDeps;
in
{
  packages = [ rosPkgs.colcon (ros.buildEnv { paths = deps; }) ];
  enterShell = ''
    echo "🔧 Welcome to the mrs_lib devenv environment!"
  '';
}

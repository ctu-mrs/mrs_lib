#!/bin/bash

while [ ! -e "build/COLCON_IGNORE" ]; do
  cd ..
  if [[ `pwd` == "/" ]]; then
    # we reached the root and didn't find the build/COLCON_IGNORE file - that's a fail!
    echo "Cannot compile, probably not in a workspace (if you want to create a new workspace, call \"colcon init\" in its root first)".
    exit 1
  fi
done

gdb -ex run ./build/mrs_lib/test/service_client_handler/test_service_client_handler

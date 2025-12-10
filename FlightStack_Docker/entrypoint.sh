#!/bin/bash
set -e

SHARED_FOLDER="/home/user/FlightstackSharedFolder"
FLIGHTSTACK_DIR="${SHARED_FOLDER}/FlightStackWorkspace/src/flightstack"
WS_DIR="${SHARED_FOLDER}/FlightStackWorkspace"

if [ -d "$FLIGHTSTACK_DIR" ]; then
  echo -e "\n✓ ACSL flightstack found"
  echo -e "To start the flightstack run:\n" 
  echo "ros2 run flightstack flightstack"
else
  echo -e "\n⚠  ACSL-flightstack not found."
  read -p "Do you want to download it now? (y/n): " answer
  case "$answer" in
    [Yy]* )
      echo -e "Cloning the repository...\n"
      mkdir -p ${WS_DIR}/src
      git clone https://github.com/andrealaffly/ACSL-flightstack ${FLIGHTSTACK_DIR}
      cd ${WS_DIR}/src/flightstack
      git submodule update --init --depth 1 --recursive -- ./include/eigen
      git submodule update --init --depth 1 --recursive -- ./include/json
      cd ${WS_DIR}
      source /opt/ros/humble/setup.bash 
      source /opt/px4_msgs_build/install/local_setup.bash
      rosdep update
      colcon build --base-paths src -r -y
      source install/local_setup.bash
      ;;
    * )
      echo -e "Download skipped. You can clone it later manually.\n"
      ;;
  esac
fi

exec "$@"
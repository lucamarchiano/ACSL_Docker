#!/bin/bash
set -e

#==========================================
# Check if flightstack is already installed
#==========================================

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

#==========================================
# Add GitHub repo info in the prompt
#==========================================
cat >> /home/user/.bashrc << 'EOF'
parse_git_repo() {
  local repo_url=$(git remote get-url origin 2>/dev/null)
  if [[ -z "$repo_url" ]]; then echo ''; return; fi
  # extract only repo name from ...user/repo(.git)
  local repo_name=$(echo "$repo_url" | sed -E 's|.*[:/]([^/:]+)/([^/]+)(\.git)?$|\2|' | sed 's/\.git$//')
  local branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
  echo "${repo_name} : ${branch}"
}

export PS1='\[\033[1;33m\]\u@\h\[\033[0m\]:\[\033[1;34m\]\w\[\033[0m\]$( [[ -n "$(git rev-parse --git-dir 2>/dev/null)" ]] && echo " \[\033[1;35m\][$(parse_git_repo)\[\033[1;35m\]]\[\033[0m\]" || echo "")\[\033[0m\]\$ \[\033[0m\]'

EOF

source /home/user/.bashrc

#========================================
# Add cleanup instructions
#========================================
cat >> /home/user/.bashrc << 'EOF'
cleanup() {
  echo ""
  echo "================================================"
  echo -e "container 'flightstack' closed\n"
  echo "To perform cleanup run stop_container.sh"
  echo "================================================"

}

trap cleanup EXIT
EOF

source /home/user/.bashrc


exec "$@"
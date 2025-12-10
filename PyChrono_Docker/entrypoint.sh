#!/bin/bash
set -e

SHARED_FOLDER="/home/user/PyChronoSharedFolder"
PYCHRONO_DIR="${SHARED_FOLDER}/UAV_Sim_PyChrono_v9.0.1"

if [ -d "$PYCHRONO_DIR" ]; then
  echo -e "\n✓ PyChrono simulator already downloaded."
  echo -e "To start the simulator run:\n" 
  echo "cd UAV_Sim_PyChrono_v9.0.1"
  echo "python main.py"
else
  echo -e "\n⚠  PyChrono simulator not found."
  read -p "Do you want to download it now? (y/n): " answer
  case "$answer" in
    [Yy]* )
      echo -e "\nCloning the repository..."
      git clone git@github.com:lucamarchiano/UAV_Sim_PyChrono_v9.0.1 $PYCHRONO_DIR
      ;;
    * )
      echo -e "\nDownload skipped. You can clone it later manually."
      ;;
  esac
fi

exec "$@"
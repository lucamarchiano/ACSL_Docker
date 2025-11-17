# Launch containers
docker compose up -d

echo -e "\nStarted container 'flightstack' with ROS2 Humble\n"

if [ -d "$HOME/DockerSharedFolder/UAV_Sim_PyChrono_v9.0.1" ]; then
  echo "PyChrono simulator already downloaded."
  echo -e "To start the simulator run:\n" 
  echo "cd UAV_Sim_PyChrono_v9.0.1"
  echo "python main.py"
else
  echo "PyChrono simulator not found."
  read -p "Do you want to download it now? (y/n): " answer
  case "$answer" in
    [Yy]* )
      echo "Cloning the repository..."
      git clone git@github.com:lucamarchiano/UAV_Sim_PyChrono_v9.0.1 ./DockerSharedFolder/UAV_Sim_PyChrono
      ;;
    * )
      echo "Download skipped. You can clone it later manually."
      ;;
  esac
fi


# Open bash shell inside container
docker compose exec uav_sim_pychrono bash

# Export host user and group id in a .env file
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env

# Allow local containers to connect to X server (needed to use Irrlicht)
xhost +local:docker

# Create the DockerSharedFolder
mkdir -p $HOME/DockerSharedFolder

# Launch containers
docker compose up -d

# Open bash shell inside container
docker compose exec uav_sim_pychrono bash

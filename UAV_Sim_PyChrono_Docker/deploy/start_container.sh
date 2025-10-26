# Export host user and group id in a .env file
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env

# Allow local containers to connect to X server (needed to use Irrlicht)
xhost +local:docker

# Create the DockerSharedFolder
mkdir -p $HOME/DockerSharedFolder

# Add configuration to run VSCode in host with container settings
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p $HOME/DockerSharedFolder/.devcontainer
cat > $HOME/DockerSharedFolder/.devcontainer/devcontainer.json << EOF
{
  "name": "UAV_Sim_PyChrono",
  "dockerComposeFile": "$SCRIPT_DIR/docker-compose.yml",
  "service": "uav_sim_pychrono",
  "workspaceFolder": "/home/user/DockerSharedFolder",
  "remoteUser": "user"
}
EOF

# Launch containers
docker compose up -d

# Open bash shell inside container
docker compose exec uav_sim_pychrono bash

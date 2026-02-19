#!/usr/bin/env bash
set -euo pipefail

#  Configurable paths 
HOST_SHARED_DIR="$HOME/PyChronoSharedFolder"
CONTAINER_SHARED_DIR="/home/user/PyChronoSharedFolder"
DEVCONTAINER_DIR="$HOST_SHARED_DIR/.devcontainer"
DOCKER_COMPOSE_FILE="docker-compose.deploy.yml"
DOCKER_COMPOSE_FILE_OVERRIDE="docker-compose.deploy.linux.yml"
DEVCONTAINER_NAME="pychrono"
CONTAINER_SERVICE_NAME="pychrono"
CONTAINER_USER="user"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Create the shared directory if it doesn't exist
if [ ! -d "${HOST_SHARED_DIR}" ]; then
    echo "Creating shared directory at ${HOST_SHARED_DIR}"
    mkdir -p "${HOST_SHARED_DIR}"
else
    echo "Shared directory already exists at ${HOST_SHARED_DIR}"
fi

# Set VSCode configuration for the devcontainer
mkdir -p "$DEVCONTAINER_DIR"

cat > "$DEVCONTAINER_DIR/devcontainer.json" << EOF
{
  "name": "$DEVCONTAINER_NAME",
  "dockerComposeFile": "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE",
  "service": "$CONTAINER_SERVICE_NAME",
  "workspaceFolder": "$CONTAINER_SHARED_DIR",
  "remoteUser": "$CONTAINER_USER",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-python.vscode-pylance",
        "ms-python.debugpy",
        "ranch-hand-robotics.rde-pack",
        "ms-vscode.cmake-tools"
      ],
      "settings": {
        "python.defaultInterpreterPath": "/usr/local/bin/python",
        "editor.semanticHighlighting.enabled": true,
        "python.languageServer": "Pylance",
        "ros.distro": "jazzy",
        "files.associations": {
          "*.launch": "xml",
          "*.xacro": "xml",
          "*.urdf": "xml",
          "*.srv": "plaintext",
          "*.msg": "plaintext",
          "*.action": "plaintext"
        }
      }
    }
  }
}
EOF

# Export host user and group id
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env

# Allow local containers to connect to X server
xhost +local:docker

# Start the container using Docker Compose
echo -e "\nStarting container '$CONTAINER_SERVICE_NAME' with PyChrono 9.0.1 and ROS2 Jazzy\n"

docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE_OVERRIDE" up -d
docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE_OVERRIDE" exec "$CONTAINER_SERVICE_NAME" /entrypoint.sh bash

# Cleanup configuration
EXIT_CODE=$?
echo ""
echo "Container '$CONTAINER_SERVICE_NAME' shell exited (code: $EXIT_CODE), but still running in background."
read -p "Kill it now with docker compose down? (y/n, default: n): " -n 1 -r choice
echo ""
if [[ $choice =~ ^[Yy]$ ]]; then
  echo "Killing '$CONTAINER_SERVICE_NAME' container..."
  docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE_OVERRIDE" down
  xhost -local:docker
  echo "Done. Container '$CONTAINER_SERVICE_NAME' killed."
else
  echo "Container '$CONTAINER_SERVICE_NAME' left running."
fi
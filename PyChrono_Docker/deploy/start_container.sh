#!/usr/bin/env bash
set -euo pipefail

#  Configurable paths 
HOST_SHARED_DIR="$HOME/PyChronoSharedFolder"
CONTAINER_SHARED_DIR="/home/user/PyChronoSharedFolder"
DEVCONTAINER_DIR="$HOST_SHARED_DIR/.devcontainer"

DOCKER_COMPOSE_BASE="docker-compose.deploy.yml"
DOCKER_COMPOSE_LINUX="docker-compose.deploy.linux.yml"

DEVCONTAINER_NAME="pychrono"
CONTAINER_SERVICE_NAME="pychrono"
CONTAINER_USER="user"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export HOST_SHARED_DIR
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

# Dynamic GPU Hardware Detection
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "✨ NVIDIA GPU detected. Using NVIDIA proprietary container layer."
    DOCKER_COMPOSE_GPU_OVERRIDE="docker-compose.deploy.linux.nvidia.yml"
else
    echo "💻 Intel/AMD Graphics detected. Using open-source DRI runtime layer."
    DOCKER_COMPOSE_GPU_OVERRIDE="docker-compose.deploy.linux.intel.yml"
fi

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
  "dockerComposeFile": [
    "$SCRIPT_DIR/$DOCKER_COMPOSE_BASE",
    "$SCRIPT_DIR/$DOCKER_COMPOSE_LINUX",
    "$SCRIPT_DIR/$DOCKER_COMPOSE_GPU_OVERRIDE"
  ],
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

# Allow local containers to connect to X server
xhost +local:docker

# Start the container using Docker Compose
echo -e "\nStarting container '$CONTAINER_SERVICE_NAME' with PyChrono 9.0.1 and ROS2 Jazzy\n"

docker compose -f "$DOCKER_COMPOSE_BASE" -f "$DOCKER_COMPOSE_LINUX" -f "$DOCKER_COMPOSE_GPU_OVERRIDE" up -d
docker compose -f "$DOCKER_COMPOSE_BASE" -f "$DOCKER_COMPOSE_LINUX" -f "$DOCKER_COMPOSE_GPU_OVERRIDE" exec "$CONTAINER_SERVICE_NAME" bash

# Cleanup configuration
EXIT_CODE=$?
echo ""
echo "Container '$CONTAINER_SERVICE_NAME' shell exited (code: $EXIT_CODE), but still running in background."
read -p "Kill it now with docker compose down? (y/n, default: n): " -n 1 -r choice
echo ""
if [[ $choice =~ ^[Yy]$ ]]; then
  echo "Killing '$CONTAINER_SERVICE_NAME' container..."
  docker compose -f "$DOCKER_COMPOSE_BASE" -f "$DOCKER_COMPOSE_LINUX" -f "$DOCKER_COMPOSE_GPU_OVERRIDE" down
  xhost -local:docker
  echo "Done. Container '$CONTAINER_SERVICE_NAME' killed."
else
  echo "Container '$CONTAINER_SERVICE_NAME' left running."
fi
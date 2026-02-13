#!/usr/bin/env bash
set -euo pipefail

# ----- configurable paths / names -----
HOST_SHARED_DIR="$HOME/PyChronoSharedFolder"
CONTAINER_SHARED_DIR="/home/user/PyChronoSharedFolder"
DEVCONTAINER_DIR="$HOST_SHARED_DIR/.devcontainer"
DOCKER_COMPOSE_FILE="docker-compose.deploy.yml"
DEVCONTAINER_NAME="pychrono"
CONTAINER_SERVICE_NAME="pychrono"
CONTAINER_USER="user"
# --------------------------------------

# Export host user and group id
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env

# Allow local containers to connect to X server
xhost +local:docker

echo "Setting up $HOST_SHARED_DIR..."

mkdir -p "$HOST_SHARED_DIR"
mkdir -p "$DEVCONTAINER_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# devcontainer.json - Python + ROS2, no hardcoding
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

echo -e "\nStarting container '$CONTAINER_SERVICE_NAME' with PyChrono 9.0.1 and ROS2 Jazzy\n"

docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" up -d
docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" exec "$CONTAINER_SERVICE_NAME" /entrypoint.sh bash

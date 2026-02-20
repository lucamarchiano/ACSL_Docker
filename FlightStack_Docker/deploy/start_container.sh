#!/usr/bin/env bash
set -euo pipefail

# Configurable paths
HOST_SHARED_DIR="$HOME/FlightstackSharedFolder"
CONTAINER_SHARED_DIR="/home/user/FlightstackSharedFolder"
DEVCONTAINER_DIR="$HOST_SHARED_DIR/.devcontainer"
DOCKER_COMPOSE_FILE="docker-compose.deploy.yml"
DOCKER_COMPOSE_FILE_OVERRIDE="docker-compose.deploy.linux.yml"
DEVCONTAINER_NAME="flightstack"
CONTAINER_SERVICE_NAME="flightstack"
CONTAINER_USER="user"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_WS_NAME="FlightStackWorkspace"
HOST_ROS_WS_DIR="$HOST_SHARED_DIR/$ROS_WS_NAME"

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
        "ms-vscode.cpptools",
        "ms-vscode.cmake-tools",
        "ranch-hand-robotics.rde-pack"
      ],
      "settings": {
        "C_Cpp.default.intelliSenseMode": "linux-gcc-x64",
        "C_Cpp.default.cppStandard": "c++17",
        "C_Cpp.default.compileCommands": "\${workspaceFolder}/$ROS_WS_NAME/build/compile_commands.json",
        "C_Cpp.default.cppFormatter": "ms-vscode.cpptools",
        "cmake.configureOnOpen": false,
        "ros.distro": "jazzy",
        "files.associations": {
          "*.launch": "xml",
          "*.xacro": "xml",
          "*.urdf": "xml",
          "*.srv": "plaintext",
          "*.msg": "plaintext",
          "*.action": "plaintext",
          "*.h": "cpp",
          "*.hpp": "cpp"
        }
      }
    }
  }
}
EOF

# Start the container using Docker Compose
echo -e "\nStarting container '$CONTAINER_SERVICE_NAME' with ROS2 Jazzy + C++/ROS extensions\n"

docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE_OVERRIDE" up -d
docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" exec "$CONTAINER_SERVICE_NAME" /entrypoint.sh bash

# # Create symlink if compile_commands.json exists
# if [[ -f "$HOST_ROS_WS_DIR/build/compile_commands.json" ]]; then
#   ln -sf "$ROS_WS_NAME/build/compile_commands.json" "$HOST_SHARED_DIR/compile_commands.json"
#   # echo "✅ Symlink: $HOST_SHARED_DIR/compile_commands.json → $HOST_ROS_WS_DIR/build/compile_commands.json"
# else
#   echo "⚠️  No compile_commands.json found. Run this inside container:"
#   echo "   cd $ROS_WS_NAME && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
# fi

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
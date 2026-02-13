#!/usr/bin/env bash
set -euo pipefail

# ----- configurable paths / names -----
HOST_SHARED_DIR="$HOME/FlightstackSharedFolder"
CONTAINER_SHARED_DIR="/home/user/FlightstackSharedFolder"
ROS_WS_NAME="FlightStackWorkspace"
HOST_ROS_WS_DIR="$HOST_SHARED_DIR/$ROS_WS_NAME"
DEVCONTAINER_DIR="$HOST_SHARED_DIR/.devcontainer"
DOCKER_COMPOSE_FILE="docker-compose.deploy.yml"
DEVCONTAINER_NAME="flightstack"
CONTAINER_SERVICE_NAME="flightstack"
CONTAINER_USER="user"
# --------------------------------------

echo "Setting up $HOST_SHARED_DIR..."

mkdir -p "$HOST_SHARED_DIR"
mkdir -p "$DEVCONTAINER_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# devcontainer.json - FIXED dockerComposeFile path to match your working original
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

echo -e "\nStarting container '$CONTAINER_SERVICE_NAME' with ROS2 Jazzy + C++/ROS extensions\n"

docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" up -d

# Create symlink if compile_commands.json exists
if [[ -f "$HOST_ROS_WS_DIR/build/compile_commands.json" ]]; then
  ln -sf "$ROS_WS_NAME/build/compile_commands.json" "$HOST_SHARED_DIR/compile_commands.json"
  # echo "✅ Symlink: $HOST_SHARED_DIR/compile_commands.json → $HOST_ROS_WS_DIR/build/compile_commands.json"
else
  echo "⚠️  No compile_commands.json found. Run this inside container:"
  echo "   cd $ROS_WS_NAME && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
fi

docker compose -f "$SCRIPT_DIR/$DOCKER_COMPOSE_FILE" exec "$CONTAINER_SERVICE_NAME" /entrypoint.sh bash

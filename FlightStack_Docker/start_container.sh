# Create the FlightstackSharedFolder
mkdir -p $HOME/FlightstackSharedFolder

# Add configuration to run VSCode in host with container settings
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p $HOME/FlightstackSharedFolder/.devcontainer
cat > $HOME/FlightstackSharedFolder/.devcontainer/devcontainer.json << EOF
{
  "name": "FlightStack",
  "dockerComposeFile": "$SCRIPT_DIR/docker-compose.deploy.yml",
  "service": "flightstack",
  "workspaceFolder": "/home/user/FlightstackSharedFolder",
  "remoteUser": "user",
  "customizations": {
  "vscode": {
    "extensions": [
      "ms-vscode.cpptools",
      "ms-vscode.cmake-tools",
      "Microsoft.vscode-ros2",
      "redhat.vscode-xml"
    ],
    "settings": {
      "C_Cpp.default.intelliSenseMode": "linux-gcc-x64",
      "C_Cpp.default.cppStandard": "c++17",
      "C_Cpp.default.compileCommands": "${workspaceFolder}/build/compile_commands.json",
      "cmake.configureOnOpen": false,
      "ros.distro": "humble",
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
EOF

echo -e "\nStarting container 'flightstack' with ROS2 Humble\n"
docker compose -f docker-compose.deploy.yml run flightstack bash
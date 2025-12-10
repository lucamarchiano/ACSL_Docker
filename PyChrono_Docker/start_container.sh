# Export host user and group id in a .env file
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env


# Allow local containers to connect to X server (needed to use Irrlicht)
xhost +local:docker


# Create the PyChronoSharedFolder
mkdir -p $HOME/PyChronoSharedFolder


# Add configuration to run VSCode in host with container settings
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p $HOME/PyChronoSharedFolder/.devcontainer
cat > $HOME/PyChronoSharedFolder/.devcontainer/devcontainer.json << EOF
{
  "name": "PyChrono",
  "dockerComposeFile": "$SCRIPT_DIR/docker-compose.deploy.yml",
  "service": "pychrono",
  "workspaceFolder": "/home/user/PyChronoSharedFolder",
  "remoteUser": "user",
  "customizations": {
  "vscode": {
    "extensions": [
      "ms-python.python",
      "ms-python.vscode-pylance",
      "ms-python.debugpy",
      "Microsoft.vscode-ros2",
      "twxs.cmake",
      "redhat.vscode-xml"
    ],
    "settings": {
      "python.defaultInterpreterPath": "/usr/local/bin/python",
      "editor.semanticHighlighting.enabled": true,
      "python.languageServer": "Pylance",
      "ros.distro": "humble",
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
EOF


echo -e "\nStarting container 'pychrono' with PyChrono 9.0.1 and ROS2 Humble\n"
docker compose -f docker-compose.deploy.yml run pychrono bash

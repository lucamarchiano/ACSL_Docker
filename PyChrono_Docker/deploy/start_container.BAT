@echo off

:: Configurable paths
set "HOST_SHARED_DIR=%USERPROFILE%\Documents\PyChronoSharedFolder"
set "CONTAINER_SHARED_DIR=/home/user/PyChronoSharedFolder"
set "DEVCONTAINER_DIR=%HOST_SHARED_DIR%\.devcontainer"

set "DOCKER_COMPOSE_BASE=docker-compose.deploy.yml"
set "DOCKER_COMPOSE_WINDOWS=docker-compose.deploy.windows.yml"

set "DEVCONTAINER_NAME=pychrono"
set "CONTAINER_SERVICE_NAME=pychrono"
set "CONTAINER_USER=user"
set "SCRIPT_DIR=%~dp0"

:: Ensure the script runs out of its own directory so paths map perfectly
cd /d "%SCRIPT_DIR%"

:: Create the shared directory if it doesn't exist
if not exist "%HOST_SHARED_DIR%" (
    echo Creating shared directory at %HOST_SHARED_DIR%
    mkdir "%HOST_SHARED_DIR%"
) else (
    echo Shared directory already exists at %HOST_SHARED_DIR%
)

:: Set VSCode configuration for the devcontainer
if not exist "%DEVCONTAINER_DIR%" mkdir "%DEVCONTAINER_DIR%"

(
echo {
echo  "name": "%DEVCONTAINER_NAME%",
echo   "dockerComposeFile": [
echo     "../%DOCKER_COMPOSE_BASE%",
echo     "../%DOCKER_COMPOSE_WINDOWS%"
echo   ],
echo  "service": "%CONTAINER_SERVICE_NAME%",
echo  "workspaceFolder": "%CONTAINER_SHARED_DIR%",
echo  "remoteUser": "%CONTAINER_USER%",
echo  "customizations": {
echo    "vscode": {
echo      "extensions": [
echo        "ms-python.python",
echo        "ms-python.vscode-pylance",
echo        "ms-python.debugpy",
echo        "ranch-hand-robotics.rde-pack",
echo        "ms-vscode.cmake-tools"
echo      ],
echo      "settings": {
echo        "python.defaultInterpreterPath": "/usr/local/bin/python",
echo        "editor.semanticHighlighting.enabled": true,
echo        "python.languageServer": "Pylance",
echo        "ros.distro": "jazzy",
echo        "files.associations": {
echo          "*.launch": "xml",
echo          "*.xacro": "xml",
echo          "*.urdf": "xml",
echo          "*.srv": "plaintext",
echo          "*.msg": "plaintext",
echo          "*.action": "plaintext"
echo        }
echo      }
echo    }
echo  }
echo }
) > "%DEVCONTAINER_DIR%\devcontainer.json"

:: Start the container using Docker Compose
echo Starting the container %CONTAINER_SERVICE_NAME% with PyChrono 9.0.1 and ROS2 Jazzy...

docker compose -f "%DOCKER_COMPOSE_BASE%" -f "%DOCKER_COMPOSE_WINDOWS%" up -d
docker compose -f "%DOCKER_COMPOSE_BASE%" -f "%DOCKER_COMPOSE_WINDOWS%" exec "%CONTAINER_SERVICE_NAME%" bash

:: Cleanup configuration
set "EXIT_CODE=%ERRORLEVEL%"
echo.
echo Container "%CONTAINER_SERVICE_NAME%" shell exited ^(code: %EXIT_CODE%^), but still running in background.
echo "Kill it now with docker compose down? (Y/N, default: N): "
choice /C YN /N
echo.
if %ERRORLEVEL%==1 (
    echo Killing "%CONTAINER_SERVICE_NAME%" container...
    docker compose -f "%DOCKER_COMPOSE_BASE%" -f "%DOCKER_COMPOSE_WINDOWS%" down
    echo Done. Container "%CONTAINER_SERVICE_NAME%" killed.
) else (
    echo Container "%CONTAINER_SERVICE_NAME%" left running.
)
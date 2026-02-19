@echo off

:: Configurable paths
set "HOST_SHARED_DIR=%USERPROFILE%\Documents\PyChronoSharedFolder"
set "CONTAINER_SHARED_DIR=/home/user/PyChronoSharedFolder"
set "DEVCONTAINER_DIR=%HOST_SHARED_DIR%\.devcontainer"
set "DOCKER_COMPOSE_FILE=docker-compose.deploy.yml"
set "DOCKER_COMPOSE_FILE_OVERRIDE=docker-compose.deploy.windows.yml"
set "DEVCONTAINER_NAME=pychrono"
set "CONTAINER_SERVICE_NAME=pychrono"
set "CONTAINER_USER=user"
set "SCRIPT_DIR=%~dp0"

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
echo     "%SCRIPT_DIR:\=/%%DOCKER_COMPOSE_FILE:\=/%",
echo     "%SCRIPT_DIR:\=/%%DOCKER_COMPOSE_FILE_OVERRIDE:\=/%"
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

:: Install VcXsrv if not already installed
winget install --id=marha.VcXsrv --exact --accept-package-agreements --accept-source-agreements --silent

:: Run VcXsrv with the appropriate settings
tasklist /fi "imagename eq vcxsrv.exe" 2>NUL | find /i "vcxsrv.exe" >NUL
if "%ERRORLEVEL%"=="0" (
    echo VcXsrv already running.
) else (
    echo Starting VcXsrv...
    start "" "C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -wgl -ac
)

:: Start the container using Docker Compose
echo Starting the container %CONTAONER_SERIVCE_NAME% with PyChrono 9.0.1 and ROS2 Jazzy...

docker compose -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE%" -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE_OVERRIDE%" up -d
docker compose -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE%" -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE_OVERRIDE%" exec "%CONTAINER_SERVICE_NAME%" /entrypoint.sh bash
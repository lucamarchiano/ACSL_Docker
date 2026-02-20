@echo off

:: Configurable paths
set "HOST_SHARED_DIR=%USERPROFILE%\Documents\FlightstackSharedFolder"
set "CONTAINER_SHARED_DIR=/home/user/FlightstackSharedFolder"
set "DEVCONTAINER_DIR=%HOST_SHARED_DIR%\.devcontainer"
set "DOCKER_COMPOSE_FILE=docker-compose.deploy.yml"
set "DOCKER_COMPOSE_FILE_OVERRIDE=docker-compose.deploy.windows.yml"
set "DEVCONTAINER_NAME=flightstack"
set "CONTAINER_SERVICE_NAME=flightstack"
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
echo      "vscode": {
echo        "extensions": [
echo          "ms-vscode.cpptools",
echo          "ms-vscode.cmake-tools",
echo          "ranch-hand-robotics.rde-pack"
echo        ],
echo        "settings": {
echo          "C_Cpp.default.intelliSenseMode": "linux-gcc-x64",
echo          "C_Cpp.default.cppStandard": "c++17",
echo          "C_Cpp.default.compileCommands": "\${workspaceFolder}/%ROS_WS_NAME%/build/compile_commands.json",
echo          "C_Cpp.default.cppFormatter": "ms-vscode.cpptools",
echo          "cmake.configureOnOpen": false,
echo          "ros.distro": "jazzy",
echo          "files.associations": {
echo            "*.launch": "xml",
echo            "*.xacro": "xml",
echo            "*.urdf": "xml",
echo            "*.srv": "plaintext",
echo            "*.msg": "plaintext",
echo            "*.action": "plaintext",
echo            "*.h": "cpp",
echo            "*.hpp": "cpp"
echo            }
echo        }
echo      }
echo   }
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
echo Starting the container %CONTAINER_SERVICE_NAME% with ROS2 Jazzy + C++/ROS extensions...

docker compose -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE%" -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE_OVERRIDE%" up -d
docker compose -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE%" -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE_OVERRIDE%" exec "%CONTAINER_SERVICE_NAME%" /entrypoint.sh bash

:: Cleanup configuration
set "EXIT_CODE=%ERRORLEVEL%"
echo.
echo Container "%CONTAINER_SERVICE_NAME%" shell exited ^(code: %EXIT_CODE%^), but still running in background.
echo "Kill it now with docker compose down? (Y/N, default: N): "
choice /C YN /N
echo.
if %ERRORLEVEL%==1 (
    echo Killing "%CONTAINER_SERVICE_NAME%" container...
    docker compose -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE%" -f "%SCRIPT_DIR%\%DOCKER_COMPOSE_FILE_OVERRIDE%" down
    echo Done. Container "%CONTAINER_SERVICE_NAME%" killed.
) else (
    echo Container "%CONTAINER_SERVICE_NAME%" left running.
)
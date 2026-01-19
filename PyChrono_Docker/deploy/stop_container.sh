#!/bin/bash

# Set script directory (optional)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Stop and remove Docker containers defined in your compose file
docker compose -f "$SCRIPT_DIR/docker-compose.deploy.yml" down

# Reset X server permissions to restrict access from local Docker containers
xhost -local:docker

echo -e "\n'pychrono' container stopped and environment reset."

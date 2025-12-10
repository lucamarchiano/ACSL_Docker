#!/bin/bash

# Set script directory (optional)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Stop and remove Docker containers defined in your compose file
docker compose -f "$SCRIPT_DIR/docker-compose.deploy.yml" down

echo -e "\n'flightstack' container stopped and environment reset."

# ACSL Docker Instructions
This guide aims at providing instructions on how to run ACSL Docker containers.
## 1. Install Docker Engine
Docker Engine is the tool that allow the user to manage and run Docker containers. It can be installed as a stand-alone CLI tool on Linux (Docker CE), or can be installed through Docker Desktop application on Windows or MacOS.
Official documentation can be found here: [Docker Engine](https://docs.docker.com/engine/install/), [Docker Desktop](https://docs.docker.com/desktop/)
### 1.2 Ubuntu Docker CE setup
To install Docker Engine on Ubuntu follow the [official guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).
To simplify the setup, an install_DockerCE.sh has been created. Simply run 
```bash
./install_DockerCE.sh
```
and verify the installation was succesful with
```bash
sudo systemctl status docker
sudo docker run hello-world
```
## 2. Docker command cheatsheet
Docker CE runs on the command line, here are reported some of the most used commands.
| Command                        | Description                                  |
|-------------------------------|----------------------------------------------|
| **Docker General Commands**   |                                              |
| docker --version              | Show Docker version                          |
| docker images                 | List all Docker images                       |
| docker ps                    | List running containers                      |
| docker ps -a                 | List all containers (running and stopped)   |
| docker pull <image>          | Download image from Docker Hub               |
| docker run <image>           | Run a container from an image                 |
| docker exec -it <container> bash | Open interactive bash shell in a running container |
| docker stop <container>      | Stop a running container                      |
| docker rm <container>        | Remove a stopped container                    |
| docker rmi <image>           | Remove a Docker image                          |
| docker logs <container>      | Show logs of a container                       |
| docker build -t <tag> .      | Build an image from a Dockerfile              |
| docker commit <container> <new_image> | Create new image from a container          |
| **Docker Compose Commands**   |                                              |
| docker compose up -d         | Start containers in detached mode             |
| docker compose down          | Stop and remove containers, networks          |
| docker compose ps            | List containers managed by Compose            |
| docker compose logs          | View logs of Compose-managed containers       |
| docker compose exec <service> bash | Run bash in a running Compose service     |
| docker compose build         | Build or rebuild services                       |
| docker compose restart       | Restart running services                        |
| **Docker Volume and Network** |                                             |
| docker volume ls             | List volumes                                   |
| docker network ls            | List networks                                  |

## 3. How to run the ACSL dockers
To run a docker container, simply run
```bash
cd PyChrono_Docker
./start_container.sh
```
>[!NOTE] 
To use the docker container, add files to the corresponding SharedFolder in order to let both the container and the host access those files.

## 4. How to manage containers image on ghcr.io
To manage container images to ghcr.io a PAT (Personal Access Token) is required.
Instructions on how to proceed can be found [here](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry)

Create PAT on GitHub personal page under Settings, Developer Settings, Personal Access Tokens
Select Write Packages and Delete Packages permissions
Copy the PAT (shown only once!)
Go to terminal and run

```bash
docker login ghcr.io
```
Use github username and the PAT as password.


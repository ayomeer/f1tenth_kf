# Docker Setup

This project uses Docker containers for running the development environment as well as for deployment (on jetson arm64 platforms). They can be build from the Dockerfiles in the .devcontainer directory or pulled from dockerhub:
- amd64: ayomeer/ros-devcontainer:galactic
- arm64: ayomeer/ros-devcontainer:arm-galactic

**Prerequisites:**
- Docker-ce (use install/docker-ce_install.sh)
    - buildx builder with linux/arm64/v8 platform (See https://docs.docker.com/reference/cli/docker/buildx/create/)


## Build System
The ros-devcontainer Dockerfile is structured as a multi-stage build with the stages `base`, `dev`, `full` and `nvidia`. 

### Build Caching
To quickly iterate on the images, it's useful to cache as much of the build as possible, such that parts that haven't changed since the last build can simply be loaded from cache, rather than being rebuilt. However, any layer content that's added from

apt-caching: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#run---mounttypecache
multi-arch caching: https://github.com/docker/buildx/discussions/1382



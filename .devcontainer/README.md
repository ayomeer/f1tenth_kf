# Docker Setup

This project uses Docker containers for running the development environment as well as for deployment (on Jetson arm64 platforms). They can be build from the Dockerfiles in this directory or pulled from Dockerhub at `ayomeer/ros-devcontainer:galactic-[build target]`. The Devcontainer Dockerfile is structured as a multi-stage build with the following stages available as build targets:

- `base`:    Minimal container for deployment with ROS2 runtime environment and not much else
- `dev`:     Adds common introspection and debuggin tools to aid during development
- `full`:    Fully featured ROS development environment meant for the development PC including Gazebo, Rviz, etc.
- `nvidia`:  Adds NVidia GPU support to accelerate graphics intensive tools

The images available on Dockerhub are built for both amd64 and arm64 architectures. Pulling one of them from either a Dev-PC or the Jetson will automatically choose the image for the matching architecture. 

> [!NOTE]
> This Docker setup uses advances features only supported by the BuildKit build engine, which is **not supported by docker.io** shipped with Ubuntu installations by default. To make BuildKit available, follow these steps:
> - Install Docker-ce using `scripts/docker-ce_install.sh` (also uninstalls any pre-existing Docker distribution beforehand)
> - Create buildx builder with linux/arm64/v8 platform (for more info see https://docs.docker.com/reference/cli/docker/buildx/create/):
>    - `docker buildx create --name arm_builder --platform linux/arm64/v8 --use`
>    - `docker buildx inspect --bootstrap`

 
For building the images, build scripts are available in the `scripts` subdirectory.
      
## Build Caching
To quickly iterate on the images, it's useful to cache as much of the build as possible, such that parts that haven't changed since the last build can simply be loaded from cache, rather than being rebuilt. Two techniques were used to improve build caching and dramatically cutting down build times:

1) [apt-caching](https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#run---mounttypecache):
By default, the lengthy process of installing packages through apt has to be repeated each time, since the builder can't be sure that this external content isn't changing between builds. By manually adding caches to the RUN commands where apt is used, we can make the builder check for this content in this cache first before downloading and installing it through apt. Here's how it works (see Dockerfile for full context):
    - Add the line `RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache` before starting to install packages through apt to disable automatic cache cleanup after finishing apt install.
    - Add the option `--mount=type=cache,id=${TARGETPLATFORM},target=/var/cache/apt,sharing=locked` to the `RUN` command, where apt is used. This specifies a cache for apt specifically, such that the packages that were installed in previous builds of the image and haven't changed, do not have to be downloaded and installed again on the next build. Specifying `id=${TARGETPLATFORM}` also distinguishes these caches between architectures, since they use differnet versions of the packages.


2) [multi-arch caching](https://github.com/docker/buildx/discussions/1382): 
Running a multi-architecture build with BuildKit using something like `docker buildx build --platform linux/amd64/,linux/arm64/v8 --push .` builds and pushes both architecture variants one after the other, and only the latest build will stay in the cache for the next build. To work around this, in the build scripts the two variants are built in seperate commands with `--cache-to` and `--cache-from` options being used to explicitly cache them as separate build caches. Finally, the standard multi-platform build command can be used to push both images to the registry, specifying both build caches to `--cache-from`, so that nothing has to be re-built.

> [!NOTE]
> Make sure the first line of the Dockerfile reads `# syntax = docker/dockerfile:1.2` to enable BuildKit caching features and that you are using BuildKit as your build engine when building the image (See note in previous section).



docker buildx build \
  --cache-from=type=registry,ref=ayomeer/ros-devcontainer:buildcache-amd64 \
  --cache-to=type=registry,ref=ayomeer/ros-devcontainer:buildcache-amd64 \
  --platform linux/amd64 \
  --target full \
  --load \
  ..

docker buildx build \
  --cache-from=type=registry,ref=ayomeer/ros-devcontainer:buildcache-amd64 \
  --platform linux/amd64 \
  --target full \
  --push \
  -t ayomeer/ros-devcontainer:galactic-full \
  ..

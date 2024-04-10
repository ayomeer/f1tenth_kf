docker buildx build \
  --cache-from=type=registry,ref=ayomeer/ros-devcontainer:buildcache-arm64 \
  --cache-to=type=registry,ref=ayomeer/ros-devcontainer:buildcache-arm64 \
  --platform linux/arm64/v8 \
  --target dev \
  --load \
  ..

docker buildx build \
  --cache-from=type=registry,ref=ayomeer/ros-devcontainer:buildcache-amd64 \
  --cache-to=type=registry,ref=ayomeer/ros-devcontainer:buildcache-amd64 \
  --platform linux/amd64 \
  --target dev \
  --load \
  ..

docker buildx build \
  --cache-from=type=registry,ref=ayomeer/ros-devcontainer:buildcache-arm64 \
  --cache-from=type=registry,ref=ayomeer/ros-devcontainer:buildcache-amd64 \
  --platform linux/amd64,linux/arm64/v8 \
  --target dev \
  --push \
  -t ayomeer/ros-devcontainer:galactic-dev \
  ..

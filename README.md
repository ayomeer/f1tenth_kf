# f1tenth_kf

## Jetson Setup
- Run through Ubuntu Setup
- Run through NVidia Jetpack Setup
- Run setup scripts for ayomeer/ros-devconainer:galactic
  
## PC Setup
- Run setup scripts for ayomeer/ros-devconainer:galactic
  
  
## Running packages in dockerized ROS 

### On Dev PC
- Clone this git repo
- Docker pull ayomeer/ros-devcontainer:galactic
- Run .devcontainer/docker-compose.yaml

# On Jetson (over SSH)
- SSH into Jetson: user@f1tenth-2.local, pw: embsw
- Clone this git repo
- Docker pull ayomeer/ros-devcontainer:arm-galactic
- cd to f1tenth_kf and enter code . to automatically start the devcontainer through vscode

Custom ROS package for improved odometry by fusion with IMU.

## Branch vesc_test


# Isaac Policy Test Gazebo

## Installation with Docker

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine) and [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

Install NVIDIA proprietary drivers if the NVIDIA graphics card should be used.

Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit) (nvidia-docker2) for NVIDIA support in the container. \
If you do not want to use NVIDIA, edit the Docker image to remove the NVIDIA section and the `run.bash` script, removing the `--gpus all` flag in the docker run command. In addition, remove the `additional_env` from the Gazebo process `gzserver` in `robot_launch/launch/robot.launch.py`.

Clone the repo with
```shell
git clone --recursive git@github.com:ddebenedittis/isaac_policy_test_gazebo.git
```
You will need to have access to the `mulinex_description` repo.

Navigate to the workspace with
```shell
cd isaac_policy_test_gazebo
```

Build the docker image (`-r` to rebuild the underlying images) :
```shell
./build.bash [-d]
```
Run the container:
```shell
./run.bash
```
Build the ROS workspace:
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash
```

## Usage

Simulate the policy on Gazebo with
```shell
ros2 launch robot_gazebo mulinex.launch.py
```

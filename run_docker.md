
Docker run — ROS2 Humble
Prerequisites
# Docker + Docker Compose v2 installed
docker --version        # >= 20.10
docker compose version  # >= 2.0
# For GUI (Gazebo/RViz)
xhost +local:docker
1. Build image
cd /home/ijud/Desktop/project/a2tech/ur3
./BUILD-DOCKER-IMAGE.sh
Or manual:
docker compose -p $USER -f ./docker/docker-compose.yml build
Build takes ~10-20 min first time (downloads base image + deps + compiles workspace).
2. Start + enter container
# CPU runtime (default)
./RUN-DOCKER.sh
# NVIDIA GPU runtime
DOCKER_RUNTIME=nvidia ./RUN-DOCKER.sh
This starts container in background, opens a bash shell inside.
3. Inside container — build workspace
Workspace auto-cloned from git in Dockerfile. If using local code via volume mount:
# Inside container
cd ~/ros2_ws
vcs import src < src/ros-ur/dependencies.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro=humble -y
colcon build --symlink-install
source install/setup.bash
4. Launch examples
Gazebo simulation (UR3 + Robotiq 85):
ros2 launch ur_gripper_gazebo ur3_cubes_example.launch.py
MoveIt2 demo:
ros2 launch ur_gripper_85_moveit_config demo.launch.py
MoveIt2 + real/sim robot:
ros2 launch ur_gripper_85_moveit_config start_moveit.launch.py ur_robot:=ur3
FT filter node:
ros2 run ur_control ft_filter -t wrench
5. Open extra shells in same container
# From host, in a new terminal
docker exec -it ${USER}-ros_ur-1 bash
# Then inside:
source ~/ros2_ws/install/setup.bash
Container name
Default: ${USER}-ros_ur-1. Override with arg:
./RUN-DOCKER.sh myproject
# → container: myproject-ros_ur-1
No NVIDIA GPU
Dockerfile already sets DOCKER_RUNTIME default to runc. Just run without the env var override:
./RUN-DOCKER.sh

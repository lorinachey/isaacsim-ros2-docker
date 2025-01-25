# isaacsim-ros2-docker
A repository for the Dockerfile and associated resources for using ROS2 with IsaacSim in a Docker container.

### Dockerfile
The Dockerfile sets up a ROS2 Humble environment with default parameters instantiated for IsaacSim's ROS2 bridge and the environment variables for running the TurtleBot3 simulation with the base controller.
For the TurtleBot3 simulation (which uses the NAV2 package), you can also modify the nav2_params.yaml file to run different controllers.
You can find the nav2_params.yaml file in `/opt/ros/humble/share/nav2_bringup/params` in the container.

### Build the Container
`docker build -t ros2-isaacsim:latest .`

### Run the Container
You can change the `ROS_DOMAIN_ID` to match the host network. Note that in order for networking to run properly between container and host, you must **NOT be running Docker in rootless mode**.

Make sure to update the run command with the correct device input for your controller, if using one.

```
docker run --rm -it --network host --ipc=host --pid=host \
  --device=/dev/input/js1 --privileged \
   -e ROS_DOMAIN_ID=0 \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  ros2-isaacsim:latest
```
#####  ⚠️ Warning

Using the `--rm` flag will automatically remove the container after it stops.  
Ensure that any important data is persisted using volumes or bind mounts, or it will be **lost permanently**.


#### Attaching to A Running Container
`docker exec -it <container name> /bin/bash`

#### Instructions Once In the Container for TurtleBot Sim

```
cd
./start_simulation.sh
```

### Starting Octomap
`ros2 launch octomap_server octomap_mapping.launch.xml`

### Troubleshooting
If you face graphical errors, such as rviz2 not starting. Make sure to run `xhost +local:docker` on the host machine.

### XBOX360 Controller
Run the following nodes:

`ros2 run joy joy_node`

`ros2 run teleop_twist_joy teleop_node --ros-args --params-file /opt/ros/humble/share/teleop_twist_joy/config/teleop_twist_joy.config.yaml`

##### Manually Setting Controller Parameters
If you need to manually change controller parameters from the config yaml.

Set enable param to the A button:
`ros2 param set /teleop_twist_joy_node enable_button 0`

To map the left stick to forward/backward (up/down) and right/left to side movement:

`ros2 param set /teleop_twist_joy_node axis_angular.yaw 0`

`ros2 param set /teleop_twist_joy_node axis_linear.x 1`


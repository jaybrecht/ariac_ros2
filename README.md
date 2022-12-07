# ARIAC ROS2 

This package is built for ROS2 Galactic running on Ubuntu 20.04 (Focal)

## Install

- Install ROS2 Galactic Desktop using the [instructions on the ROS2 wiki](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html#)

- Create a colcon workspace 

```
$ source /opt/ros/galactic/setup.bash
$ mkdir -p ~/ariac_ws/src
$ cd ~/ariac_ws
```

- Clone this repository into the src directory of the ariac_ws

```
$ git clone https://github.com/jaybrecht/ariac_ros2.git src/ariac_ros2
```

- Install dependencies

```
$ rosdep install --from-paths src -y --ignore-src
```

- Build the colcon workspace

```
$ colcon build
```

## Starting the Environment

To launch the ARIAC environment:

```
$ . ~/ariac_ws/install/setup.bash
$ ros2 launch ariac_gazebo ariac.launch.py
```

To launch the ARIAC environment with MoveIt and RVIZ

```
$ . ~/ariac_ws/install/setup.bash
$ ros2 launch ariac_gazebo ariac.launch.py start_moveit:=true start_rviz:=true
```
## Running the test competitor

Launch the ARIAC environment:

```
$ . ~/ariac_ws/install/setup.bash
$ ros2 launch ariac_gazebo ariac.launch.py start_moveit:=true
```

In another terminal start the test competitor:

```
$ . ~/ariac_ws/install/setup.bash
$ ros2 launch test_competitor moveit_test.launch.py
```

## Controlling the mobile robot

Launch the ARIAC environment:

```
$ . ~/ariac_ws/install/setup.bash
$ ros2 launch ariac_gazebo ariac.launch.py
```

In another terminal send twist commands:

```
$ ros2 topic pub --rate 30 /mobile_robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

To use the keyboard teleop:

```
$ sudo apt install ros-galactic-turtlebot3-teleop
$ export TURTLEBOT3_MODEL=waffle
$ ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/mobile_robot
```
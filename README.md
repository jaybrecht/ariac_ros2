# ARIAC ROS2 

## To start the environment with RVIZ

Launch the ARIAC environment:

`ros2 launch ariac_gazebo ariac.launch.py start_moveit:=true start_rviz:=true`

## To run the test competitor

Launch the ARIAC environment:

`ros2 launch ariac_gazebo ariac.launch.py start_moveit:=true`

Launch the test competitor:

`ros2 launch test_competitor moveit_test.launch.py`
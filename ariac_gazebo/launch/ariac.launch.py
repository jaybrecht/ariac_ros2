import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Set the path to this package.
    pkg_share = FindPackageShare(package='ariac_gazebo').find('ariac_gazebo')

    # Set the path to the world file
    world_file_name = 'ariac.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    # General arguments
    start_rviz = LaunchConfiguration("start_rviz")
    start_moveit = LaunchConfiguration("start_moveit")
    start_slam = LaunchConfiguration("start_slam", default="false")
    start_navigation = LaunchConfiguration("start_navigation", default="false")

    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Floor Robot Bringup
    floor_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch",
             "/floor_robot_bringup.launch.py"]
        ),
        launch_arguments={'start_moveit': start_moveit,
                          'start_rviz': start_rviz}.items()
    )

    # Ceiling Robot Bringup
    ceiling_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch",
             "/ceiling_robot_bringup.launch.py"]
        ),
        launch_arguments={'start_moveit': start_moveit,
                          'start_rviz': start_rviz}.items()
    )

    # AGV Bringup
    agv1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch",
             "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv1", 'y_position': "4.8"}.items()
    )

    agv2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch",
             "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv2", 'y_position': "1.2"}.items()
    )

    agv3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch",
             "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv3", 'y_position': "-1.2"}.items()
    )

    agv4_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch",
             "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv4", 'y_position': "-4.8"}.items()
    )

    # Mobile Robot Bringup
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_human"), "/launch",
             "/start_navigation.launch.py"]
        ),
        condition=IfCondition(start_navigation),
    )
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_human"), "/launch",
             "/start_slam.launch.py"]
        ),
        condition=IfCondition(start_slam),
    )


    sensor_spawner = Node(
        package='ariac_gazebo',
        executable='spawn_sensors.py',
        output='screen',
        arguments=[]
    )

    tray_spawner = Node(
        package='ariac_gazebo',
        executable='spawn_trays.py',
        output='screen',
        arguments=[]
    )

    sensor_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='sensor_tf_broadcaster.py',
        output='screen',
        arguments=[]
    )

    object_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='object_tf_broadcaster.py',
        output='screen',
        arguments=[]
    )
    
    # ROSbridge
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [FindPackageShare("rosbridge_server"), "/launch", "/rosbridge_websocket_launch.xml"]
        ),
    )

    # Robot Spawner Node
    robot_spawner = Node(
        package='ariac_gazebo',
        executable='spawn_robots.py',
        output='screen',
        arguments=[]
    )
    
      # Clock node
    clockNode = Node(
        package='ariac_human',
        executable='clockNode.py',
        output='screen',
        arguments=[]
    )

    # Human listener node
    human_listener = Node(
        package='ariac_human',
        executable='human_listener.py',
        output='screen',
        arguments=[]
    )
    
    # Robot listener node
    robot_listener = Node(
        package='ariac_human',
        executable='robot_listener.py',
        output='screen',
        arguments=[]
    )
    
    # Snapshot node
    snapshot = Node(
        package='ariac_human',
        executable='snapshot.py'
    )


    # movebase_server node
    movebase_server = Node(
        package='ariac_human',
        executable='movebase_server.py'
    )


    spawn_robots_after_sensors = RegisterEventHandler(
        OnProcessExit(
            target_action=sensor_spawner,
            on_exit=[
                robot_spawner,
                agv1_bringup,
                agv2_bringup,
                agv3_bringup,
                agv4_bringup,
                floor_robot_bringup,
                ceiling_robot_bringup,
                human_listener,
        	robot_listener,
        	snapshot,
        	movebase_server
            ]
        )
    )
    
    
    nodes_to_start = [
        gazebo,
        tray_spawner,
        sensor_spawner,
        sensor_tf_broadcaster,
        object_tf_broadcaster,
        navigation,
        slam,
        spawn_robots_after_sensors,
        clockNode,
        rosbridge
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("start_rviz",
                              default_value="false",
                              description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument("start_moveit",
                              default_value="false",
                              description="Start moveit nodes for the robots?")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_slam",
            default_value="false",
            description="Start slam?")
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_navigation",
            default_value="false",
            description="Start navigation?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

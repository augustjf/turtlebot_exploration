import launch
import launch_ros.actions
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtle_exploration'), 'launch')

    #Exploration
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_file_dir, '/turtle_exploration.launch.py')])
    )
    #Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_gazebo"), '/launch', '/turtlebot3_house.launch.py'])   
    )

    #SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_cartographer"), '/launch', '/cartographer.launch.py']),
            launch_arguments={'use_sim_time': 'True'}.items()         
    )

    # order = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=slam,
    #         on_start=[explore],
    #     )
    # )

    return LaunchDescription([
        explore,
        #simulation,
        #slam
    ])

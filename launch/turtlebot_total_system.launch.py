import os
import launch.actions
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
   
    #Exploration
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtle_exploration'), 'launch'),
         '/turtle_exploration.launch.py'])
    )
    
    #Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_gazebo"), '/launch', '/turtlebot3_big_house.launch.py'])   
    )

    #SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_cartographer"), '/launch', '/cartographer.launch.py']),
            launch_arguments={'use_sim_time': 'True'}.items()         
    )

    #Timer to wait for simulator
    exploration_timer = launch.actions.TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="Timer Finished"),
            explore
        ]
    )

    return LaunchDescription([   
        simulation,
        slam,
        exploration_timer
        ])

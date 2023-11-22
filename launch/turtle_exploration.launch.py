import launch
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Exploration
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=    'turtle_exploration',
            namespace=  'turtle_exploration',
            executable= 'turtle_exploration',
            name='explore',
            output='screen',
        )
    ])
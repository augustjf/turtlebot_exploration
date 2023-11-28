import launch
import launch_ros.actions


def generate_launch_description():
    #Exploration
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=    'turtle_exploration',
            executable= 'turtle_exploration',
            name='explore'
        )
    ])
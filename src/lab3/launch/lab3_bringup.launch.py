from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'
        ),
        Node(
            package='lab3',
            namespace='Eater_oshi',
            executable='eater.py',
            name='EaterTurtle',
            parameters = [
                {'sampling_frequency' : 100.00}
            ]
        ),
        Node(
            package='lab3',
            namespace='Killer_oshi',
            executable='killer.py',
            name='KillerTurtle',
            parameters = [
                {'sampling_frequency' : 100.00}
            ]
        ),
        # Node(
        #     package='lab3',
        #     namespace='',
        #     executable='killer.py',
        #     name='KillerTurtle',
        #     parameters = [
        #         {'sampling_frequency' : 100}
        #     ]
        # ),
        # Node(
        #     package='lab3',
        #     namespace='',
        #     executable='killer.py',
        #     name='KillerTurtle',
        #     parameters = [
        #         {'sampling_frequency' : 0.01}
        #     ]
        # ),
        # Node(
        #     package='turtlesim',
        #     executable='mimic',
        #     name='mimic',
        #     remappings=[
        #         ('/input/pose', '/turtlesim1/turtle1/pose'),
        #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     ]
        # )
    ])
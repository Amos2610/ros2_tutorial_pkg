"""
課題５: Turtlebot3で自律走行させるプログラムを作成せよ．また，それを起動するLaunchファイルを作成せよ．
"""
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Launching autonomous_move_turtlebot3 node."),
        
        # # ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         '/home/cassis-orange2/fumoto_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py'
        #     )
        # ),

        # ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/fumoto_ws/map.yaml 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/home/cassis-orange2/fumoto_ws/src/turtlebot3/turtlebot3_navigation2/launch/navigation2.launch.py'
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'map': '/home/cassis-orange2/fumoto_ws/map.yaml'
            }.items(),
        ),

        # ros2 run ros2_tutorial_pkg autonomous_move_turtlebot3
        Node(
            package='ros2_tutorial_pkg',
            executable='autonomous_move_turtlebot3',
            name='autonomous_move_turtlebot3',
            output='screen'
        )
    ])

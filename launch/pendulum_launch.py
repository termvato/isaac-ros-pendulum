from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

# Replays a recorded run in RViz, so the results can be seen without Isaac Sim.
# The bag supplies /joint_states and /clock; robot_state_publisher turns the
# joint states into /tf, and RViz draws the URDF from that.

# __file__ is the path to THIS launch file, as Python was given it.
# realpath() makes it absolute and resolves any symlinks, so it is the same
# answer no matter which directory you ran `ros2 launch` from.
# dirname() then strips "pendulum_launch.py" off the end, leaving the folder.
here = os.path.dirname(os.path.realpath(__file__))

# Build paths relative to that folder rather than hardcoding absolute ones.
# Clone the repo somewhere else and these still resolve.
urdf_path = os.path.join(here, '..', 'urdf', 'body.urdf')
rviz_config = os.path.join(here, '..', 'rviz', 'pendulum.rviz')
default_bag = os.path.join(here, '..', 'bags', 'sweep_inf')

# robot_state_publisher wants the URDF's TEXT, not its path, so read the file.
# This runs at import time, before any node starts, so a bad path fails here
# with a plain Python traceback rather than inside a launched process.
with open(urdf_path) as f:
    robot_description = f.read()


def generate_launch_description():
    # LaunchConfiguration is a placeholder filled in when launch runs, not a
    # string, so it can go into a command list but not into an f-string.
    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value=default_bag,
                              description='bag folder to replay'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='playback speed; 1.0 replays at recording '
                                          'pace (~0.3x real time), ~3.3 is real time'),

        # Parses the URDF, subscribes to /joint_states, publishes /tf.
        # use_sim_time so its transforms carry the bag's sim-time stamps.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]
        ),
        # The bag is the only /joint_states publisher. No --clock flag: that
        # would publish the bag's receive times (wall clock), which disagree
        # with the sim-time header stamps. The recorded /clock is replayed
        # instead, so every stamp and the clock agree.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--rate', rate],
            output='screen'
        ),
    ])

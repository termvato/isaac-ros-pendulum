from launch import LaunchDescription
from launch_ros.actions import Node
import os

# __file__ is the path to THIS launch file, as Python was given it.
# realpath() makes it absolute and resolves any symlinks, so it is the same
# answer no matter which directory you ran `ros2 launch` from.
# dirname() then strips "pendulum_launch.py" off the end, leaving the folder.
here = os.path.dirname(os.path.realpath(__file__))

# Build the URDF path relative to that folder rather than hardcoding an
# absolute path. Clone the repo somewhere else and this still resolves.
# os.path.join handles the separator so you never glue strings with "/".
# NOTE: this assumes body.urdf sits NEXT TO this file. It currently does not.
urdf_path = os.path.join(here, '..', 'urdf', 'body.urdf')

# robot_state_publisher wants the URDF's TEXT, not its path, so read the file.
# `with` closes the file automatically, even if reading raises.
# This runs at import time, i.e. before any node starts, so a bad path fails
# here with a plain Python traceback rather than inside a launched process.
with open(urdf_path) as f:
    robot_description = f.read()

# setting up the configuration for rviz to show base_link as the fixed frame 
# and display the robot model
rviz_config = os.path.join(here, '..', 'rviz', 'pendulum.rviz')


def generate_launch_description():
    # launch calls this function and expects a LaunchDescription back:
    # a list of actions to perform. Each Node action starts one process.
    return LaunchDescription([
        # Parses the URDF, subscribes to /joint_states, publishes /tf.
        # No `name` set, so it defaults to /robot_state_publisher.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # parameters is a LIST of sources; each dict maps name -> value.
            # 'robot_description' is the name this node looks for; it is fixed.
            parameters=[{'robot_description': robot_description}]
        ),
        # Sliders, one per non-fixed joint found in the robot description.
        # Publishes /joint_states. Stands in for the real controller for now.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            # ros_arguments wraps these in --ros-args for you.
            # warn silences this node's chatty INFO output.
            ros_arguments=['--log-level', 'warn']
        ),
        # Visualisation. Starts with default config, so Fixed Frame is 'map'
        # and there is no RobotModel display until you add one by hand.
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])

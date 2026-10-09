"""Learned controller: runs a PPO policy exported from Isaac Lab (rl/) as ONNX.

Same interface as pendulum_controller.py (sub /joint_states, pub /joint_command, 120 Hz,
stale-state guard), so the harness and the plant cannot tell the two apart.

    ros2 run pendulum_nodes pendulum_policy --ros-args --params-file config/limits.yaml \
        -p model:=/abs/path/policy.onnx -p use_sim_time:=true
"""

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState


class PendulumPolicy(Node):

    def __init__(self):
        super().__init__('pendulum_policy')
        self.latest_state = None

        # Policy outputs a unitless action in [-1, 1]; training scaled it by max_torque,
        # so the deployed scale must be the same number. No default: inf would be wrong.
        self.declare_parameter('max_torque', float('nan'))
        self.max_torque = self.get_parameter('max_torque').value
        if not np.isfinite(self.max_torque):
            raise RuntimeError('max_torque not set; pass --params-file config/limits.yaml')

        self.declare_parameter('model', '')
        model = self.get_parameter('model').value
        if not model:
            raise RuntimeError('model not set; pass -p model:=/abs/path/policy.onnx')
        # The exported graph includes the observation normalizer, so it takes raw states.
        self.session = ort.InferenceSession(model, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f'loaded {model}, input "{self.input_name}" '
                               f'{self.session.get_inputs()[0].shape}')

        self.max_age = 0.05

        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.timer = self.create_timer(1.0 / 120.0, self.timer_callback)
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.latest_state = msg

    def timer_callback(self):
        if self.latest_state is None:
            return

        joints = JointState()
        joints.name = ['body_arm']
        state = self.latest_state

        age = (self.get_clock().now() - Time.from_msg(state.header.stamp)).nanoseconds * 1e-9
        if age > self.max_age:
            # Same reasoning as the LQR node: Isaac holds the last command, so publish zero.
            joints.effort = [0.0]
            self.get_logger().warn(
                f'/joint_states stale by {age:.3f} s, commanding zero effort',
                throttle_duration_sec=1.0)
        else:
            # Observation order must match rl/pendulum_env.py _get_observations exactly.
            obs = np.array([[
                state.position[state.name.index('base_body')],
                state.velocity[state.name.index('base_body')],
                state.velocity[state.name.index('body_arm')],
            ]], dtype=np.float32)
            action = float(self.session.run(None, {self.input_name: obs})[0][0, 0])
            # Deterministic: the ONNX graph is the mean action, no exploration noise.
            action = max(-1.0, min(1.0, action))  # same clip as the env's _pre_physics_step
            joints.effort = [action * self.max_torque]

        joints.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(joints)


def main(args=None):
    rclpy.init(args=args)
    node = PendulumPolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

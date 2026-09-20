import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

class PendulumController(Node):

    def __init__(self):
        super().__init__('pendulum_controller')
        self.latest_state = None
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        timer_period = 1.0/120.0  # 120Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            JointState,
            'joint_states', 
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.latest_state = msg

    def timer_callback(self):
        if self.latest_state is None:
            return
        joints = JointState()
        joints.name = ['body_arm']
        joints.effort =[0.0]
        joints.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(joints)
        self.get_logger().info(f'body_arm:{joints.effort[0]:.2f}')


def main(args=None):
    rclpy.init(args=args)

    pendulum_controller = PendulumController()

    rclpy.spin(pendulum_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pendulum_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

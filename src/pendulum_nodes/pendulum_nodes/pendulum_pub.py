import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState


class PendulumPub(Node):

    def __init__(self):
        super().__init__('pendulum_pub')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        joints = JointState()
        joints.name = ['body_arm', 'base_body']
        joints.position = [self.i, float(math.sin(math.pi*self.i))]
        joints.velocity =[0.0, 0.0]
        joints.effort =[0.0, 0.0]
        joints.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(joints)
        self.get_logger().info(f'body_arm:{joints.position[0]:.2f} base_body:{joints.position[1]:.2f}')
        self.i += 0.05


def main(args=None):
    rclpy.init(args=args)

    pendulum_pub = PendulumPub()

    rclpy.spin(pendulum_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pendulum_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

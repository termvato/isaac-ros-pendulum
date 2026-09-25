import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState

class PendulumController(Node):

    def __init__(self):
        super().__init__('pendulum_controller')
        self.latest_state = None

        self.declare_parameter('max_torque', float('inf'))
        self.max_torque = self.get_parameter('max_torque').value

        # LQR gains, derived in LQR_example.py from the linearised model about
        # upright. State is x = [theta_base, omega_base, omega_arm], input is the
        # torque on body_arm. Control law is u = -K x; the signs below already
        # absorb that negation, since every entry of K came out negative.
        #   Q from Bryson's rule (budgets: 0.785 rad, pi rad/s, 5 rad/s), R = 1e5.
        # Closed-loop poles: -33.7 and -6.36 +/- 0.04j, so tau_c = 0.157 s.
        # Region of attraction is limited by arm saturation, not by gains:
        # saturation starts at 0.079 rad (4.5 deg);
        # recovery still succeeds to ~0.142 rad (8.2 deg)
        self.declare_parameter('k_theta_base', 9.16291716e-01)
        self.k_theta_base = self.get_parameter('k_theta_base').value
        self.declare_parameter('k_w_base', 1.43911551e-01)
        self.k_w_base = self.get_parameter('k_w_base').value
        self.declare_parameter('k_w_arm', 6.32455532e-04)
        self.k_w_arm = self.get_parameter('k_w_arm').value

        self.max_age = 0.05 

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

        state=self.latest_state

        now   = self.get_clock().now()
        stamp = Time.from_msg(state.header.stamp)
        age   = (now - stamp).nanoseconds * 1e-9

        if age > self.max_age:
            # State is stale, so the gains would be acting on an angle that is no
            # longer true. Publish zero rather than returning: Isaac holds the last
            # command forever, so going quiet would leave the last torque applied.
            joints.effort=[0.0]
            self.get_logger().warn(
                f'/joint_states stale by {age:.3f} s, commanding zero effort',
                throttle_duration_sec=1.0)
        else:
            Ob=state.position[state.name.index('base_body')]
            wb=state.velocity[state.name.index('base_body')]
            wa=state.velocity[state.name.index('body_arm')]

            u = (self.k_theta_base * Ob
                + self.k_w_base * wb
                + self.k_w_arm * wa)
            
            
            u = max(-self.max_torque, min(self.max_torque, u)) #limiting the torque
            

            joints.effort =[u]


        joints.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(joints)
        self.get_logger().info(f'body_arm:{joints.effort[0]:.5f}')


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

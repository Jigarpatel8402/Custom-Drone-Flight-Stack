import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DroneVelocity
from drone_interfaces.msg import DroneState
from drone_interfaces.msg import DroneThrottle

class AltitudeVelocityController(Node):
    def __init__(self):
        super().__init__('altitude_velocity_control')
        self.counter = 0.0
        self.desired_vel_z = 0.0
        self.current_vel_z = 0.0

        self.intergral_error = 0.0
        self.previous_error = 0.0

        self.declare_parameter('hover_throttle', 350.0)
        self.hover_throttle = self.get_parameter('hover_throttle').value
        self.declare_parameter('kp_altitude_velocity', 100.0)
        self.kp_altitude_velocity = self.get_parameter('kp_altitude_velocity').value
        self.declare_parameter('ki_altitude_velocity', 10.0)
        self.ki_altitude_velocity = self.get_parameter('ki_altitude_velocity').value
        self.declare_parameter('kd_altitude_velocity', 20.0)
        self.kd_altitude_velocity = self.get_parameter('kd_altitude_velocity').value

        self.velocity_sub = self.create_subscription(
            DroneVelocity,
            '/desired_velocity',
            self.velocity_callback,
            10
        )

        self.state_sub = self.create_subscription(
            DroneState,
            '/drone_state',
            self.state_callback,
            10
        )

        self.throttle_pub = self.create_publisher(
            DroneThrottle,
            '/drone_throttle',
            10
        )

        self.create_timer(
            0.02,
            self.control_loop
        )
        self.dt = 0.02
    
    def velocity_callback(self, msg):
        self.desired_vel_z = msg.vel_z
    
    def state_callback(self, msg):
        self.current_vel_z = msg.linear_vel.z
    
    def control_loop(self):
        altitude_velocity_error = self.desired_vel_z - self.current_vel_z

        self.intergral_error += altitude_velocity_error * self.dt
        self.intergral_error = max(
            min(self.intergral_error, 2.0),
            -2.0
        )

        derivative_error = (
            altitude_velocity_error - self.previous_error
        ) / self.dt

        self.previous_error = altitude_velocity_error

        throttle = (
            self.hover_throttle
            + self.kp_altitude_velocity * altitude_velocity_error
            + self.ki_altitude_velocity * self.intergral_error
            + self.kd_altitude_velocity * derivative_error
        )
        throttle = max(
            min(throttle, 600.0),
            100.0
        )

        throttle_msg = DroneThrottle()
        throttle_msg.throttle = throttle

        self.throttle_pub.publish(throttle_msg)

        # self.counter += 1
        # if self.counter % 25 == 0:
        #     self.get_logger().info(
        #         f"P={self.kp_altitude_velocity * altitude_velocity_error:.4f}, "
        #         f"I={self.ki_altitude_velocity * self.intergral_error:.4f}, "
        #         f"D={self.kd_altitude_velocity * derivative_error:.4f}, "
        #         f"thr={throttle:.4f}"
        #     )


def main(args = None):
    rclpy.init(args = args)
    node = AltitudeVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

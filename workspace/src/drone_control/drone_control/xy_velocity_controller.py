import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DroneVelocity, DroneState, DroneAttitude

class XYVelocityController(Node):
    def __init__(self):
        super().__init__('xy_velocity_control')
        self.counter = 0
        self.desired_vel_x = 0.0
        self.desired_vel_y = 0.0
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0

        self.declare_parameter('kp_xy_velocity', 3.0)
        self.kp_xy_velocity = self.get_parameter('kp_xy_velocity').value
        self.declare_parameter('max_tilt', 10.0)
        self.max_tilt = self.get_parameter('max_tilt').value

        self.velocity_sub = self.create_subscription(
            DroneVelocity,
            '/desired_velocity',
            self.velocity_callback,
            10
        )

        self.state_sub = self.create_subscription(
            DroneState,
            'drone_state',
            self.state_callback,
            10
        )

        self.attitude_pub = self.create_publisher(
            DroneAttitude,
            '/desired_attitude',
            10
        )

        self.create_timer(
            0.02,
            self.control_loop
        )
        self.dt = 0.02
    
    def velocity_callback(self, msg):
        self.desired_vel_x = msg.vel_x
        self.desired_vel_y = msg.vel_y
    
    def state_callback(self, msg):
        self.current_vel_x = msg.linear_vel.x
        self.current_vel_y = msg.linear_vel.y
    
    def control_loop(self):
        vel_x_error = self.desired_vel_x - self.current_vel_x
        vel_y_error = self.desired_vel_y - self.current_vel_y

        desired_pitch = -self.kp_xy_velocity * vel_x_error
        desired_roll = self.kp_xy_velocity * vel_y_error

        desired_pitch = max(
            min(desired_pitch, self.max_tilt), 
            -self.max_tilt
        )
        desired_roll = max(
            min(desired_roll, self.max_tilt),
            -self.max_tilt
        )

        attitude_msg = DroneAttitude()
        attitude_msg.roll = desired_roll
        attitude_msg.pitch = desired_pitch
        attitude_msg.yaw = 0.0

        self.attitude_pub.publish(attitude_msg)

        self.counter += 1
        if self.counter % 125 == 0:
            self.get_logger().info(
                f"vx_des={self.desired_vel_x:.2f}, "
                f"vx={self.current_vel_x:.2f}, "
                f"pitch_des={desired_pitch:.2f}, "
                f"vy_des={self.desired_vel_y:.2f}, "
                f"vy={self.current_vel_y:.2f}, "
                f"roll_des={desired_roll:.2f}"
            )


def main(args = None):
    rclpy.init(args = args)
    node = XYVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

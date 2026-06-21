import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DronePose
from drone_interfaces.msg import DroneVelocity

class PositionController(Node):
    def __init__(self):
        super().__init__('position_control')
        self.counter = 0
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.desired_vel_x = 0.0
        self.desired_vel_y = 0.0
        self.desired_vel_z = 0.0

        self.z_integral = 0.0

        self.declare_parameter('kp_position', 1.0)
        self.kp_position = self.get_parameter('kp_position').value
        self.declare_parameter('ki_position', 0.1)
        self.ki_position = self.get_parameter('ki_position').value
        self.declare_parameter('max_velocity', 2.0)
        self.max_velocity = self.get_parameter('max_velocity').value

        self.pose_sub = self.create_subscription(
            DronePose,
            '/drone_pose',
            self.pose_callback,
            10
        )

        self.setpoint_sub = self.create_subscription(
            DronePose,
            '/drone_setpoint',
            self.setpoint_callback,
            10
        )

        self.velocity_pub = self.create_publisher(
            DroneVelocity,
            '/desired_velocity',
            10
        )

        self.create_timer(
            0.02,
            self.control_loop
        )
        self.dt = 0.02
        
    def setpoint_callback(self, msg):
        self.desired_x = msg.x
        self.desired_y = msg.y
        self.desired_z = msg.z
        self.z_integral = 0.0
    
    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
    
    def control_loop(self):
        x_error = self.desired_x - self.current_x
        y_error = self.desired_y - self.current_y
        z_error = self.desired_z - self.current_z

        self.z_integral += z_error * self.dt
        self.z_integral = max(
            min(self.z_integral, 1.0),
            -1.0
        )

        self.desired_vel_x = (
            self.kp_position * x_error
        )
        self.desired_vel_y = (
            self.kp_position * y_error
        )
        self.desired_vel_z = (
            self.kp_position * z_error
            + self.ki_position * self.z_integral
        )

        self.desired_vel_x = max(
            min(
                self.desired_vel_x, 
                self.max_velocity), 
            -self.max_velocity
        )
        self.desired_vel_y = max(
            min(
                self.desired_vel_y, 
                self.max_velocity), 
            -self.max_velocity
        )
        self.desired_vel_z = max(
            min(
                self.desired_vel_z, 
                self.max_velocity), 
            -self.max_velocity
        )

        velocity_msg = DroneVelocity()
        velocity_msg.vel_x = self.desired_vel_x
        velocity_msg.vel_y = self.desired_vel_y
        velocity_msg.vel_z = self.desired_vel_z

        self.velocity_pub.publish(velocity_msg)

        self.counter += 1
        if (self.counter % 20 == 0):
            self.get_logger().info(
                f"x_des={self.desired_x:.2f}, "
                f"x={self.current_x:.2f}, "
                f"err={x_error:.2f}, "
                f"vx_des={self.desired_vel_x:.2f}"
            )

def main(args = None):
    rclpy.init(args = args)
    node = PositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

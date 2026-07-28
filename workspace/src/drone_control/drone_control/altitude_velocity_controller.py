import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DroneVelocity, DroneState, DroneCollective
from drone_model.vehicle_model import Vehicle
from drone_model.constants import Physics

class AltitudeVelocityController(Node):
    def __init__(self):
        super().__init__('altitude_velocity_control')
        self.counter = 0.0
        self.desired_vel_z = 0.0
        self.current_vel_z = 0.0

        self.integral_error = 0.0
        self.previous_error = 0.0

        self.collective_force = 0.0

        self.declare_parameter('kp_altitude_velocity', 5.0)
        self.kp_altitude_velocity = self.get_parameter('kp_altitude_velocity').value
        self.declare_parameter('ki_altitude_velocity', 0.5)
        self.ki_altitude_velocity = self.get_parameter('ki_altitude_velocity').value
        self.declare_parameter('kd_altitude_velocity', 1.0)
        self.kd_altitude_velocity = self.get_parameter('kd_altitude_velocity').value

        self.declare_parameter('max_accel', 5.0)
        self.max_accel = self.get_parameter('max_accel').value

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

        self.collective_pub = self.create_publisher(
            DroneCollective,
            '/control_collective',
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

        if (0.0 < self.collective_force < Vehicle.MAX_COLLECTIVE_THRUST):
            self.integral_error += altitude_velocity_error * self.dt
        self.integral_error = max(
            min(self.integral_error, 2.0),
            -2.0
        )

        derivative_error = (
            altitude_velocity_error - self.previous_error
        ) / self.dt
        
        self.previous_error = altitude_velocity_error

        desired_accel = (
            self.kp_altitude_velocity * altitude_velocity_error +
            self.ki_altitude_velocity * self.integral_error +
            self.kd_altitude_velocity * derivative_error
        )

        desired_accel = max(
            -self.max_accel,
            min(self.max_accel, desired_accel)
        )

        self.collective_force = Vehicle.MASS * (
            Physics.GRAVITY +
            desired_accel
        )
        self.collective_force = max(
            0.0,
            min(Vehicle.MAX_COLLECTIVE_THRUST, self.collective_force),
        )
        
        collective_msg = DroneCollective()
        collective_msg.force= self.collective_force
        
        self.collective_pub.publish(collective_msg)

        self.counter += 1
        # if self.counter % 20 == 0:
        #     self.get_logger().info(
        #         f"vz_des={self.desired_vel_z:.4f}, "
        #         f"vz={self.current_vel_z:.4f}, "
        #         f"error={altitude_velocity_error:.4f}, "
        #         f"derivative_error={derivative_error:.4f}, "
        #         f"P={self.kp_altitude_velocity * altitude_velocity_error:.4f}, "
        #         f"I={self.ki_altitude_velocity * self.integral_error:.4f}, "
        #         f"D={self.kd_altitude_velocity * derivative_error:.4f}, "
        #         f"PID={desired_accel:.4f}, "
        #         f"collective={self.collective_force:.4f}"
        #     )


def main(args = None):
    rclpy.init(args = args)
    node = AltitudeVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

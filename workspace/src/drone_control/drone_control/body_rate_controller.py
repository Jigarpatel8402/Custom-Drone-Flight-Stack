import rclpy
from rclpy.node import Node
from drone_model.constants import Physics
from drone_model.modes import ArmState, ControlMode
from drone_interfaces.msg import DroneState, DroneAttitudeRate, DroneMode, DroneControl

class BodyRateController(Node):
    def __init__(self):
        super().__init__('rate_control')
        self.counter = 0
        self.desired_roll_rate = 0.0
        self.desired_pitch_rate = 0.0
        self.desired_yaw_rate = 0.0

        self.current_roll_rate = 0.0
        self.current_pitch_rate = 0.0
        self.current_yaw_rate = 0.0

        self.roll_rate_integral = 0.0
        self.pitch_rate_integral = 0.0
        self.yaw_rate_integral = 0.0

        self.previous_roll_rate = 0.0
        self.previous_pitch_rate = 0.0
        self.previous_yaw_rate = 0.0

        self.arm_state = ArmState.DISARMED
        self.control_mode = ControlMode.IDLE
        
        self.declare_parameter('kp_roll_rate', 1.0)
        self.kp_roll_rate = self.get_parameter('kp_roll_rate').value
        self.declare_parameter('ki_roll_rate', 0.0)
        self.ki_roll_rate = self.get_parameter('ki_roll_rate').value
        self.declare_parameter('kd_roll_rate', 0.0)
        self.kd_roll_rate = self.get_parameter('kd_roll_rate').value

        self.declare_parameter('kp_pitch_rate', 0.05)
        self.kp_pitch_rate = self.get_parameter('kp_pitch_rate').value
        self.declare_parameter('ki_pitch_rate', 0.0)
        self.ki_pitch_rate = self.get_parameter('ki_pitch_rate').value
        self.declare_parameter('kd_pitch_rate', 0.0)
        self.kd_pitch_rate = self.get_parameter('kd_pitch_rate').value
        
        self.declare_parameter('kp_yaw_rate', 0.03)
        self.kp_yaw_rate = self.get_parameter('kp_yaw_rate').value
        self.declare_parameter('ki_yaw_rate', 0.0)
        self.ki_yaw_rate = self.get_parameter('ki_yaw_rate').value
        self.declare_parameter('kd_yaw_rate', 0.0)
        self.kd_yaw_rate = self.get_parameter('kd_yaw_rate').value

        self.declare_parameter('max_roll_integral', 10.0)
        self.max_roll_integral = (
            self.get_parameter('max_roll_integral').value * Physics.DEG2RAD
        )
        self.declare_parameter('max_pitch_integral', 10.0)
        self.max_pitch_integral = (
            self.get_parameter('max_pitch_integral').value * Physics.DEG2RAD
        )
        self.declare_parameter('max_yaw_integral', 15.0)
        self.max_yaw_integral = (
            self.get_parameter('max_yaw_integral').value * Physics.DEG2RAD
        )

        self.declare_parameter('max_roll_output', 1.0)
        self.max_roll_output = self.get_parameter('max_roll_output').value
        self.declare_parameter('max_pitch_output', 1.0)
        self.max_pitch_output = self.get_parameter('max_pitch_output').value
        self.declare_parameter('max_yaw_output', 0.3)
        self.max_yaw_output = self.get_parameter('max_yaw_output').value

        self.state_sub = self.create_subscription(
            DroneState,
            '/drone_state',
            self.state_callback,
            10
        )

        self.rate_sub = self.create_subscription(
            DroneAttitudeRate,
            '/desired_rates',
            self.rate_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            DroneMode,
            '/drone_mode',
            self.mode_callback,
            10
        )
        
        self.attitude_pub = self.create_publisher(
            DroneControl,
            '/control_rates',
            10
        )
        
        self.create_timer(
            0.004,
            self.control_loop
        )
        self.dt = 0.004

    def state_callback(self, msg):
        self.current_roll_rate = msg.angular_vel.x
        self.current_pitch_rate = msg.angular_vel.y
        self.current_yaw_rate = msg.angular_vel.z
        
    def rate_callback(self, msg):
        self.desired_roll_rate = msg.roll_rate
        self.desired_pitch_rate = msg.pitch_rate
        self.desired_yaw_rate = msg.yaw_rate
    
    def mode_callback(self, msg):
        if self.arm_state != msg.arm_state or self.control_mode != msg.control_mode:
            self.roll_rate_integral = 0.0
            self.pitch_rate_integral = 0.0
            self.yaw_rate_integral = 0.0

            self.previous_roll_rate = 0.0
            self.previous_pitch_rate = 0.0
            self.previous_yaw_rate = 0.0
        
        self.arm_state = msg.arm_state
        self.control_mode = msg.control_mode
    
    def control_loop(self):
        roll_rate_error = self.desired_roll_rate - self.current_roll_rate
        pitch_rate_error = self.desired_pitch_rate - self.current_pitch_rate
        yaw_rate_error = self.desired_yaw_rate - self.current_yaw_rate

        roll_rate_derivative = (
            self.current_roll_rate - 
            self.previous_roll_rate
        ) / self.dt
        pitch_rate_derivative = (
            self.current_pitch_rate - 
            self.previous_pitch_rate
        ) / self.dt
        yaw_rate_derivative = (
            self.current_yaw_rate - 
            self.previous_yaw_rate
        ) / self.dt

        roll_output = (
            self.kp_roll_rate * roll_rate_error +
            self.ki_roll_rate * self.roll_rate_integral -
            self.kd_roll_rate * roll_rate_derivative
        )
        pitch_output = (
            self.kp_pitch_rate * pitch_rate_error +
            self.ki_pitch_rate * self.pitch_rate_integral -
            self.kd_pitch_rate * pitch_rate_derivative
        )
        yaw_output = (
            self.kp_yaw_rate * yaw_rate_error +
            self.ki_yaw_rate * self.yaw_rate_integral -
            self.kd_yaw_rate * yaw_rate_derivative
        )

        if abs(roll_output) < self.max_roll_output:
            self.roll_rate_integral += roll_rate_error * self.dt
        if abs(pitch_output) < self.max_pitch_output:            
            self.pitch_rate_integral += pitch_rate_error * self.dt
        if abs(yaw_output) < self.max_yaw_output:
            self.yaw_rate_integral += yaw_rate_error * self.dt
        
        # self.roll_rate_integral = max(
        #     -self.max_roll_integral,
        #     min(self.max_roll_integral, self.roll_rate_integral)
        # )
        # self.pitch_rate_integral = max(
        #     -self.max_pitch_integral,
        #     min(self.max_pitch_integral, self.pitch_rate_integral)
        # )
        # self.yaw_rate_integral = max(
        #     -self.max_yaw_integral,
        #     min(self.max_yaw_integral, self.yaw_rate_integral)
        # )

        roll_output = max(
            min(roll_output, self.max_roll_output),
            -self.max_roll_output
        )
        pitch_output = max(
            min(pitch_output, self.max_pitch_output),
            -self.max_pitch_output
        )
        yaw_output = max(
            min(yaw_output, self.max_yaw_output),
            -self.max_yaw_output
        )

        self.previous_roll_rate = self.current_roll_rate
        self.previous_pitch_rate = self.current_pitch_rate
        self.previous_yaw_rate = self.current_yaw_rate

        attitude_msg = DroneControl()
        attitude_msg.roll = roll_output
        attitude_msg.pitch = pitch_output
        attitude_msg.yaw = yaw_output
        
        self.attitude_pub.publish(attitude_msg)

        self.counter += 1
        if self.counter % 100 == 0:
            self.get_logger().info(
                f"Desired_yaw_Rate={self.desired_yaw_rate:.4f}, "
                f"CurrentYawRate={self.current_yaw_rate:.4f}, "
                f"P={self.kp_yaw_rate * yaw_rate_error:.4f}, "
                f"I={self.ki_yaw_rate * self.yaw_rate_integral:.4f}, "
                f"D={self.kd_yaw_rate * yaw_rate_derivative:.4f}, "
                f"yaw_output={yaw_output:.4f}"
                # f"desired_pitch_rate={self.desired_pitch_rate:.4f}, "
                # f"current_pitch_rate={self.current_pitch_rate:.4f}, "
                # f"pitch_output={pitch_output:.4f}, "
                # f"desired_yaw_rate={self.desired_yaw_rate:.4f}, "
                # f"current_yaw_rate={self.current_yaw_rate:.4f}, "
                # f"yaw_output={yaw_output:.4f}\n"
            )
            # self.get_logger().info(
            # f"Desired Pitch Rate={self.desired_pitch_rate:.4f}, "
            # f"Current pitch Rate={self.current_pitch_rate:.4f}, "
            # f"Rate Error={pitch_rate_error:.4f}, "
            # f"PitchOutput={pitch_output:.4f}"
            # )
            # self.get_logger().info(
            #     f"DesiredYawRate={self.desired_yaw_rate:.4f}, "
            #     f"CurrentYawRate={self.current_yaw_rate:.4f}, "
            #     f"YawRateError={yaw_rate_error:.4f}, "
            #     f"YawOutput={yaw_output:.4f}"
            # )


def main(args = None):
    rclpy.init(args = args)
    node = BodyRateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

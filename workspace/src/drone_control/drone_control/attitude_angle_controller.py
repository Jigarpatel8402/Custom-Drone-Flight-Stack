import rclpy
from rclpy.node import Node
from drone_model.constants import Physics
from drone_model.modes import ArmState, ControlMode
from drone_interfaces.msg import DroneAttitude, DronePose, DroneMode, DroneAttitudeRate

class AttitudeAngleController(Node):
    def __init__(self):
        super().__init__('attitude_control')
        self.counter = 0
        self.i = 100
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0
        
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        self.yaw_integral = 0.0

        self.previous_roll_error = 0.0
        self.previous_pitch_error = 0.0
        self.previous_yaw_error = 0.0

        self.max_roll_reached = 0.0
        self.max_pitch_reached = 0.0
        self.max_yaw_reached = 0.0

        self.arm_state = ArmState.DISARMED
        self.control_mode = ControlMode.IDLE
        
        self.declare_parameter('kp_roll', 4.0)
        self.kp_roll = self.get_parameter('kp_roll').value
        self.declare_parameter('ki_roll', 0.0)
        self.ki_roll = self.get_parameter('ki_roll').value
        self.declare_parameter('kd_roll', 0.0)
        self.kd_roll = self.get_parameter('kd_roll').value

        self.declare_parameter('kp_pitch', 2.0)
        self.kp_pitch = self.get_parameter('kp_pitch').value
        self.declare_parameter('ki_pitch', 0.0)
        self.ki_pitch = self.get_parameter('ki_pitch').value
        self.declare_parameter('kd_pitch', 0.0)
        self.kd_pitch = self.get_parameter('kd_pitch').value
        
        self.declare_parameter('kp_yaw', 2.0)
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.declare_parameter('ki_yaw', 0.0)
        self.ki_yaw = self.get_parameter('ki_yaw').value
        self.declare_parameter('kd_yaw', 0.0)
        self.kd_yaw = self.get_parameter('kd_yaw').value

        self.declare_parameter('max_integral', 10.0)
        self.max_integral = (
            self.get_parameter('max_integral').value * Physics.DEG2RAD
        )

        self.declare_parameter('max_roll_rate', 30.0)
        self.max_roll_rate = (
            self.get_parameter('max_roll_rate').value * Physics.DEG2RAD
        )
        self.declare_parameter('max_pitch_rate', 30.0)
        self.max_pitch_rate = (
            self.get_parameter('max_pitch_rate').value * Physics.DEG2RAD
        )
        self.declare_parameter('max_yaw_rate', 10.0)
        self.max_yaw_rate = (
            self.get_parameter('max_yaw_rate').value * Physics.DEG2RAD
        )
        
        self.pose_sub = self.create_subscription(
            DronePose,
            '/drone_pose',
            self.pose_callback,
            10
        )

        # self.attitude_sub = self.create_subscription(
        #     DroneAttitude,
        #     '/desired_attitude',
        #     self.attitude_callback,
        #     10
        # )

        self.setpoint_sub = self.create_subscription(
            DronePose,
            '/drone_setpoint',
            self.setpoint_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            DroneMode,
            '/drone_mode',
            self.mode_callback,
            10
        )

        self.rate_pub = self.create_publisher(
            DroneAttitudeRate,
            '/desired_rates',
            10
        )

        self.create_timer(
            0.004,
            self.control_loop
        )
        self.dt = 0.004

        self.get_logger().info("attitude control started")
    
    # def attitude_callback(self, msg):
    #     self.desired_roll = msg.roll
    #     self.desired_pitch = msg.pitch
    
    def setpoint_callback(self, msg):
        self.desired_roll = msg.roll
        self.desired_pitch = msg.pitch
        self.desired_yaw = msg.yaw
    
    def pose_callback(self, msg):
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw
    
    def mode_callback(self, msg):
        if self.arm_state != msg.arm_state or self.control_mode != msg.control_mode:
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            self.yaw_integral = 0.0

            self.previous_roll_error = 0.0
            self.previous_pitch_error = 0.0
            self.previous_yaw_error = 0.0
        
        self.arm_state = msg.arm_state
        self.control_mode = msg.control_mode
    
    def AbsAngle(self, ang):
        return ((ang + Physics.PI) % (2 * Physics.PI)) - Physics.PI
    
    def control_loop(self):
        roll_error = self.AbsAngle(self.desired_roll - self.current_roll)
        pitch_error = self.AbsAngle(self.desired_pitch - self.current_pitch)
        yaw_error = self.AbsAngle(self.desired_yaw - self.current_yaw)

        roll_derivative = (roll_error - self.previous_roll_error) / self.dt
        pitch_derivative = (pitch_error - self.previous_pitch_error) / self.dt
        yaw_derivative = (yaw_error - self.previous_yaw_error) / self.dt
        
        desired_roll_rate = (
            self.kp_roll * roll_error + 
            self.ki_roll * self.roll_integral + 
            self.kd_roll * roll_derivative
        )
        desired_pitch_rate = (
            self.kp_pitch * pitch_error + 
            self.ki_pitch * self.pitch_integral + 
            self.kd_pitch * pitch_derivative
        )
        desired_yaw_rate = (
            self.kp_yaw * yaw_error + 
            self.ki_yaw * self.yaw_integral + 
            self.kd_yaw * yaw_derivative
        )

        if abs(desired_roll_rate) < self.max_roll_rate:
            self.roll_integral += roll_error * self.dt
        if abs(desired_pitch_rate) < self.max_pitch_rate:
            self.pitch_integral += pitch_error * self.dt
        if abs(desired_yaw_rate) < self.max_yaw_rate:
            self.yaw_integral += yaw_error * self.dt
        
        # self.roll_integral = max(
        #     -self.max_integral,
        #     min(self.max_integral, self.roll_integral)
        # )
        # self.pitch_integral = max(
        #     -self.max_integral,
        #     min(self.max_integral, self.pitch_integral)
        # )
        # self.yaw_integral = max(
        #     -self.max_integral,
        #     min(self.max_integral, self.yaw_integral)
        # )

        desired_roll_rate = max(
            min(desired_roll_rate, self.max_roll_rate), 
            -self.max_roll_rate
        )
        desired_pitch_rate = max(
            min(desired_pitch_rate, self.max_pitch_rate), 
            -self.max_pitch_rate
        )
        desired_yaw_rate = max(
            min(desired_yaw_rate, self.max_yaw_rate), 
            -self.max_yaw_rate
        )
        
        self.previous_roll_error = roll_error
        self.previous_pitch_error = pitch_error
        self.previous_yaw_error = yaw_error

        if self.max_roll_reached < self.current_roll:
            self.max_roll_reached = self.current_roll
        if self.max_pitch_reached < self.current_pitch:
            self.max_pitch_reached = self.current_pitch
        if self.max_yaw_reached < self.current_yaw:
            self.max_yaw_reached = self.current_yaw
        
        rate_msg = DroneAttitudeRate()
        rate_msg.roll_rate = desired_roll_rate
        rate_msg.pitch_rate = desired_pitch_rate
        rate_msg.yaw_rate = desired_yaw_rate
        
        self.rate_pub.publish(rate_msg)
        
        self.counter += 1
        if self.counter % 100 == 0:
            self.get_logger().info(
                f"desired_roll={self.desired_roll:.4f}, "
                f"current_roll={self.current_roll:.4f}, "
                f"max_roll_reached={self.max_roll_reached}, "
                f"desired_pitch={self.desired_pitch:.4f}, "
                f"current_pitch={self.current_pitch:.4f}, "
                f"max_pitch_reached={self.max_pitch_reached:.4f}, "
                f"desired_yaw={self.desired_yaw:.4f}, "
                f"current_yaw={self.current_yaw:.4f}, "
                f"max_yaw_reached={self.max_yaw_reached}, "
                f"desired_yaw_rate={desired_yaw_rate}"
            )


def main(args = None):
    rclpy.init(args=args)
    node = AttitudeAngleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

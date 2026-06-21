import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DronePose
from drone_interfaces.msg import DroneAttitude

class AttitudeController(Node):
    def __init__(self):
        super().__init__('attitude_control')
        self.counter = 0
        self.i = 100
        self.throttle = 0.0
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
        
        self.declare_parameter('kp_roll', 2.0)
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

        self.declare_parameter('max_attitude_output', 5.0)
        self.max_attitude_output = self.get_parameter('max_attitude_output').value

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

        self.attitude_pub = self.create_publisher(
            DroneAttitude,
            '/control_attitude',
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
        self.throttle = msg.throttle
        
    def pose_callback(self, msg):
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw
    
    def AbsAngle(self, ang):
        return ((ang + 180) % 360) - 180 

    def control_loop(self):
        roll_error = self.AbsAngle(self.desired_roll - self.current_roll)
        pitch_error = self.AbsAngle(self.desired_pitch - self.current_pitch)
        yaw_error = self.AbsAngle(self.desired_yaw - self.current_yaw)

        self.roll_integral += roll_error * self.dt
        self.pitch_integral += pitch_error * self.dt
        self.yaw_integral += yaw_error * self.dt
        
        roll_derivative = (roll_error - self.previous_roll_error) / self.dt
        pitch_derivative = (pitch_error - self.previous_pitch_error) / self.dt
        yaw_derivative = (yaw_error - self.previous_yaw_error) / self.dt

        roll_output = (
            self.kp_roll * roll_error + 
            self.ki_roll * self.roll_integral + 
            self.kd_roll * roll_derivative
        )
        pitch_output = (
            self.kp_pitch * pitch_error + 
            self.ki_pitch * self.pitch_integral + 
            self.kd_pitch * pitch_derivative
        )
        yaw_output = (
            self.kp_yaw * yaw_error + 
            self.ki_yaw * self.yaw_integral + 
            self.kd_yaw * yaw_derivative
        )

        roll_output = max(
            min(roll_output, self.max_attitude_output), 
            -self.max_attitude_output
        )
        pitch_output = max(
            min(pitch_output, self.max_attitude_output), 
            -self.max_attitude_output
        )
        yaw_output = max(
            min(yaw_output, self.max_attitude_output), 
            -self.max_attitude_output
        )
        
        self.previous_roll_error = roll_error
        self.previous_pitch_error = pitch_error
        self.previous_yaw_error = yaw_error

        attitude_msg = DroneAttitude()
        attitude_msg.roll = roll_output
        attitude_msg.pitch = pitch_output
        attitude_msg.yaw = yaw_output
        attitude_msg.throttle = self.throttle

        self.attitude_pub.publish(attitude_msg)

        self.counter += 1
        if self.counter % 100 == 0:
            self.get_logger().info(
                f"Current Roll = {self.current_roll:.4f}, "
                f"Current pitch = {self.current_pitch:.4f}, "
                f"Current yaw = {self.current_yaw:.4f}, "
                f"Desired Roll = {self.desired_roll:.4f}, "
                f"Desired yaw = {self.desired_yaw:.4f}, "
                f"Roll Error = {roll_error:.4f}, "
                f"Roll Output = {roll_output:.4f}, "
                f"yaw Error = {yaw_error:.4f}, "
                f"yaw output = {yaw_output:.4f}"
            )
            self.get_logger().info(
                f"dt={self.dt:.6f}, "
                f"error={roll_error:.4f}, "
                f"P_output={self.kp_roll * roll_error}, "
                f"I_output={self.ki_roll * self.roll_integral}, "
                f"D_output={self.kd_roll * roll_derivative:.4f}"
            )

        # str = f"Error = {roll_error:.2f}, D = {roll_derivative:.2f}, P_out = {self.kp_roll * roll_error:.2f}, D_out = {self.kd_roll * roll_derivative:.2f}, I = {self.roll_integral:.2f}, I_out = {self.ki_roll * self.roll_integral}"
        
        # if self.counter % 400 == 0 and self.desired_roll != 0.0 and self.counter <= 20000:
        #     self.get_logger().info(
        #         f"""{str}
        #         Current Roll = {current_roll:.2f}, Desired Roll = {self.desired_roll:.2f}, Roll Error = {roll_error:.2f}, Roll Output = {roll_output:.2f}"""
        #     )
        #     self.i = self.i - 1

def main(args = None):
    rclpy.init(args=args)
    node = AttitudeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DroneThrottle
from drone_interfaces.msg import DroneAttitude
from drone_interfaces.msg import DroneRotor, DronePose

class MotorMixer(Node):
    def __init__(self):
        super().__init__('motor_mixer')
        self.counter = 0.0
        self.throttle = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # self.throttle_sub = self.create_subscription(
        #     DroneThrottle,
        #     '/drone_throttle',
        #     self.throttle_callback,
        #     10
        # )

        # self.attitude_sub = self.create_subscription(
        #     DroneAttitude,
        #     '/control_attitude',
        #     self.attitude_callback,
        #     10
        # )

        self.setpoint_sub = self.create_subscription(
            DronePose,
            '/drone_setpoint',
            self.setpoint_callback,
            10
        )
        
        self.motor_pub = self.create_publisher(
            DroneRotor,
            '/motor_commands',
            10
        )

        self.create_timer(
            0.004,
            self.control_loop
        )
        self.dt = 0.004
        
        self.get_logger().info(
            'Motor Mixer started'
        )

    # def throttle_callback(self, msg):
    #     self.throttle = msg.throttle
    
    # def attitude_callback(self, msg):
    #     self.roll = msg.roll
    #     self.pitch = msg.pitch
    #     self.yaw = msg.yaw
    #     self.throttle = msg.throttle

    def setpoint_callback(self, msg):
        self.throttle = msg.throttle
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw
    
    def control_loop(self):
        motor0 = self.throttle + self.roll - self.pitch - self.yaw
        motor1 = self.throttle - self.roll - self.pitch + self.yaw
        motor2 = self.throttle - self.roll + self.pitch - self.yaw
        motor3 = self.throttle + self.roll + self.pitch + self.yaw

        motor_msg = DroneRotor()
        motor_msg.rotor0 = motor0
        motor_msg.rotor1 = motor1
        motor_msg.rotor2 = motor2
        motor_msg.rotor3 = motor3

        self.motor_pub.publish(motor_msg)

        self.counter += 1

        # if self.counter % 100 == 0:
        #     self.get_logger().info(
        #         f'''m1 = {motor0:.2f}, m2 = {motor1:.2f}, m3 = {motor2:.2f}, m4 = {motor3:.2f}'''
        #     )


def main(args = None):
    rclpy.init(args = args)
    node = MotorMixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

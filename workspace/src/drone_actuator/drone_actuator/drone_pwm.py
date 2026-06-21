import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DroneRotor

class DronePWM(Node):
    def __init__(self):
        super().__init__('drone_pwm')
        self.counter = 0
        self.rotor0 = 0.0
        self.rotor1 = 0.0
        self.rotor2 = 0.0
        self.rotor3 = 0.0

        self.declare_parameter('timer', 0.0)               # need to change when uses in real drones
        self.timer = self.get_parameter('timer').value
        
        self.motor_sub = self.create_subscription(
            DroneRotor,
            '/motor_commands',
            self.motor_callback,
            10
        )

        self.pwm_pub = self.create_publisher(
            DroneRotor,
            '/pwm_output',
            10
        )

        self.create_timer(
            0.004,
            self.control_loop
        )
        self.dt = 0.004

        self.get_logger().info("drone pwm started")
    
    def motor_callback(self, msg):
        self.rotor0 = msg.rotor0
        self.rotor1 = msg.rotor1
        self.rotor2 = msg.rotor2
        self.rotor3 = msg.rotor3

    def control_loop(self):
        pwm_msg = DroneRotor()
        pwm_msg.rotor0 = 1000 + self.rotor0
        pwm_msg.rotor1 = 1000 + self.rotor1
        pwm_msg.rotor2 = 1000 + self.rotor2
        pwm_msg.rotor3 = 1000 + self.rotor3

        self.pwm_pub.publish(pwm_msg)

        # self.counter += 1

        # if self.counter == 0 and int(pwm_msg.rotor0) >= 1000:
        #     self.counter += 1
        #     self.get_logger().info(
        #         f'''m0 = {msg.rotor0:.2f}, m1 = {msg.rotor1:.2f}, m2 = {msg.rotor2:.2f}, m3 = {msg.rotor3:.2f}'''
        #     )
        #     self.get_logger().info(
        #         f'''pwm0={pwm_msg.rotor0}, pwm1={pwm_msg.rotor1}, pwm2={pwm_msg.rotor2}, pwm3={pwm_msg.rotor3}'''
        #     )


def main(args = None):
    rclpy.init(args = args)
    node = DronePWM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

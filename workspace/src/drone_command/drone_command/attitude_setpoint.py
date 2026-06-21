import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading

class AttitudeSetPoint(Node):
    def __init__(self):
        super().__init__('drone_setpoint')
        self.desired_throttle = 0.0
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0

        self.attitude_pub = self.create_publisher(
            Float32MultiArray,
            '/attitude_setpoint',
            10
        )

        self.get_logger().info("Publisher Created")

        self.timer = self.create_timer(
            0.05,
            self.publish_setpoint
        )

        self.input_thread = threading.Thread(
            target = self.input_call,
            daemon = True
        )
        self.input_thread.start()

    def publish_setpoint(self):
        msg = Float32MultiArray()
        msg.data = [
            self.desired_throttle, 
            self.desired_roll, 
            self.desired_pitch, 
            self.desired_yaw
        ]
        self.attitude_pub.publish(msg)

    def input_call(self):
        while rclpy.ok():
            cmd = input(">> ").split(" ")
            self.desired_throttle = float(cmd[0])
            self.desired_roll = float(cmd[1])
            self.desired_pitch = float(cmd[2])
            self.desired_yaw = float(cmd[3])

def main(args = None):
    rclpy.init(args = args)
    node = AttitudeSetPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.get_logger().info(
            'IMU Node has been started.'
            )
        
        self.subscription = self.create_subscription(
            Imu,
            '/world/iris_runway/model/iris/link/imu_link/sensor/imu_sensor/imu',
            self.imu_callback,
            10
        )
    
    def imu_callback(self, msg):
        self.get_logger().info(
            f'''Angular Velocity X Y Z = ({msg.angular_velocity.x:.3f}, {msg.angular_velocity.y:.3f}, {msg.angular_velocity.z:.3f})
             Linear Acceleration X Y Z = ({msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f})
             Quaternion Orientation X Y Z W = ({msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f})'''
        )

def main(args=None):
    rclpy.init(args = args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

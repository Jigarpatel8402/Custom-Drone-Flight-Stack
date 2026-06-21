import rclpy
from rclpy.node import Node
import math
from drone_interfaces.msg import DroneIMU
from drone_interfaces.msg import DroneBarometer
from drone_interfaces.msg import DroneGPS
from drone_interfaces.msg import DronePose
from drone_interfaces.msg import DroneState

class EstimatorNode(Node):
    def __init__(self):
        super().__init__('estimator_node')
        self.declare_parameter('accel_weight', 0.02)
        self.accel_weight = self.get_parameter('accel_weight').value
        self.declare_parameter('gyro_weight', 0.98)
        self.gyro_weight = self.get_parameter('gyro_weight').value
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.reference_pressure = None
        self.EARTH_RADIUS = 6378137.0
        
        self.previous_time = None
        self.counter = 0
        self.i = 20

        self.pose_msg = DronePose()
        self.state_msg =DroneState()
        
        self.get_logger().info(
            'Estimator Node has been started.'
        )
        
        self.IMU_sub = self.create_subscription(
            DroneIMU,
            '/drone_imu',
            self.imu_callback,
            10
        )

        self.Barometer_sub = self.create_subscription(
            DroneBarometer,
            'drone_barometer',
            self.barometer_callback,
            10
        )

        self.GPS_sub = self.create_subscription(
            DroneGPS,
            '/drone_gps',
            self.gps_callback,
            10
        )
        
        self.pose_pub = self.create_publisher(
            DronePose,
            '/drone_pose',
            10
        )

        self.state_pub = self.create_publisher(
            DroneState,
            '/drone_state',
            10
        )
    
    def barometer_callback(self, msg):
        if self.reference_pressure is None:
            self.reference_pressure = msg.pressure
            return
        self.altitude = 44330.0 * (
            1.0 - (msg.pressure / self.reference_pressure) ** 0.1903
        )
    
    def gps_callback(self, msg):
        self.vel_x = msg.velocity_north
        self.vel_y = msg.velocity_east
        self.vel_z = msg.velocity_up

        if self.home_lat == 0.0:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            return
        
        lat_rad = math.radians(msg.latitude)
        home_lat_rad = math.radians(self.home_lat)

        d_lat = math.radians(msg.latitude - self.home_lat)
        d_lon = math.radians(msg.longitude - self.home_lon)
        
        self.latitude = self.EARTH_RADIUS * d_lat
        self.longitude = (
            self.EARTH_RADIUS * math.cos(home_lat_rad) * d_lon
        )
    
    def imu_callback(self, msg):
        self.current_time = self.get_clock().now()

        if self.previous_time == None:
            self.previous_time = self.current_time
            return
        
        dt = (self.current_time - self.previous_time).nanoseconds / 1e9
        self.previous_time = self.current_time
        
        gyro_x = msg.gyro_x
        gyro_y = -msg.gyro_y
        gyro_z = -msg.gyro_z

        ax = msg.accel_x
        ay = msg.accel_y
        az = msg.accel_z

        acc_roll = math.atan2(-ay, -az)
        acc_pitch = math.atan2(
            ax, 
            math.sqrt(ay * ay + az * az)
        )

        gyro_roll = self.roll + gyro_x * dt
        gyro_pitch = self.pitch + gyro_y * dt

        self.roll = (
            self.gyro_weight * gyro_roll + 
            self.accel_weight * acc_roll
        )
        self.pitch = (
            self.gyro_weight * gyro_pitch + 
            self.accel_weight * acc_pitch
        )
        self.yaw += gyro_z * dt
        
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        yaw_deg = math.degrees(self.yaw)

        # roll_deg = ((roll_deg + 180) % 360) - 180
        # pitch_deg = ((pitch_deg + 180) % 360) - 180
        # yaw_deg = ((yaw_deg + 180) % 360) - 180
        
        self.pose_msg.roll = roll_deg
        self.pose_msg.pitch = pitch_deg
        self.pose_msg.yaw = yaw_deg
        self.pose_msg.x = self.latitude
        self.pose_msg.y = self.longitude
        self.pose_msg.z = self.altitude

        # vertical_accel = az + (9.81 * math.cos(self.roll) * math.cos(self.pitch))
        
        # self.vel_z *= 0.995
        # self.vel_z += vertical_accel * dt

        self.state_msg.linear_vel.x = self.vel_x
        self.state_msg.linear_vel.y = self.vel_y
        self.state_msg.linear_vel.z = self.vel_z
        self.state_msg.angular_vel.x = gyro_x
        self.state_msg.angular_vel.y = gyro_y
        self.state_msg.angular_vel.z = gyro_z

        self.pose_pub.publish(self.pose_msg)
        self.state_pub.publish(self.state_msg)

        self.counter += 1
        
        # if self.counter % 50 == 0:
        #     self.i -= 1
        #     self.get_logger().info(
        #         f"current Roll Pitch Yaw = ({roll_deg}, {pitch_deg}, {yaw_deg})"
        #     )
        
        # if self.counter % 50 == 0:
        #     self.get_logger().info(
        #         f"vz={self.vel_z:.3f}"
        #     )


def main(args = None):
    rclpy.init(args=args)
    node = EstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

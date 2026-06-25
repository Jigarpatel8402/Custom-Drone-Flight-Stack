import rclpy
from rclpy.node import Node
import math
from drone_interfaces.msg import DroneIMU
from drone_interfaces.msg import DroneBarometer
from drone_interfaces.msg import DroneMagnetometer
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
        self.declare_parameter('mag_yaw_weight', 0.02)
        self.mag_yaw_weight = self.get_parameter('mag_yaw_weight').value
        self.declare_parameter('gyro_yaw_weight', 0.98)
        self.gyro_yaw_weight = self.get_parameter('gyro_yaw_weight').value
        
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
        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z =0.0
        self.mag_initialize = False
        self.mag_yaw_reference = 0.0

        self.filtered_gyro_x = 0.0
        self.filtered_gyro_y = 0.0
        self.filtered_gyro_z = 0.0
        
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
            '/drone_barometer',
            self.barometer_callback,
            10
        )

        self.Magnetometer_sub = self.create_subscription(
            DroneMagnetometer,
            '/drone_magnetometer',
            self.magnetometer_callback,
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
    
    def magnetometer_callback(self, msg):
        self.mag_x = msg.magnetic_x
        self.mag_y = msg.magnetic_y
        self.mag_z = msg.magnetic_z
        if not self.mag_initialize:
            self.mag_yaw_reference = math.atan2(self.mag_y, self.mag_x)
            self.mag_initialize = True
    
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

    def AngleDiff(self, target, current):
        return ((target - current + math.pi) % (2 * math.pi)) - math.pi
    
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

        accel_roll = math.atan2(-ay, -az)
        accel_pitch = -math.atan2(
            ax, 
            math.sqrt(ay * ay + az * az)
        )

        gyro_roll = self.roll + gyro_x * dt
        gyro_pitch = self.pitch + gyro_y * dt
        gyro_yaw = self.yaw + gyro_z * dt
        
        mag_xh = (
            self.mag_x * math.cos(self.pitch) + 
            self.mag_z * math.sin(self.pitch)
        )
        mag_yh = (
            self.mag_x * math.sin(self.roll) * math.sin(self.pitch)
            + self.mag_y * math.cos(self.roll)
            - self.mag_z * math.sin(self.roll) * math.cos(self.pitch)
        )
        
        mag_yaw = -math.atan2(mag_yh, mag_xh)
        mag_yaw += self.mag_yaw_reference
        yaw_error = self.AngleDiff(mag_yaw, gyro_yaw)
        
        self.roll = (
            self.gyro_weight * gyro_roll + 
            self.accel_weight * accel_roll
        )
        self.pitch = (
            self.gyro_weight * gyro_pitch + 
            self.accel_weight * accel_pitch
        )
        self.yaw = gyro_yaw + 0.01 * yaw_error
        # self.yaw = gyro_yaw

        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        yaw_deg = math.degrees(self.yaw)

        self.filtered_gyro_x = (
            0.9 * self.filtered_gyro_x +
            0.1 * gyro_x
        )

        self.filtered_gyro_y = (
            0.9 * self.filtered_gyro_y +
            0.1 * gyro_y
        )

        self.filtered_gyro_z = (
            0.9 * self.filtered_gyro_z +
            0.1 * gyro_z
        )
        
        self.pose_msg.roll = roll_deg
        self.pose_msg.pitch = pitch_deg
        self.pose_msg.yaw = yaw_deg
        self.pose_msg.x = self.latitude
        self.pose_msg.y = self.longitude
        self.pose_msg.z = self.altitude

        self.state_msg.linear_vel.x = self.vel_x
        self.state_msg.linear_vel.y = self.vel_y
        self.state_msg.linear_vel.z = self.vel_z
        self.state_msg.angular_vel.x = math.degrees(self.filtered_gyro_x)
        self.state_msg.angular_vel.y = math.degrees(self.filtered_gyro_y)
        self.state_msg.angular_vel.z = math.degrees(self.filtered_gyro_z)

        self.pose_pub.publish(self.pose_msg)
        self.state_pub.publish(self.state_msg)

        mag_yaw_deg = math.degrees(mag_yaw)

        self.counter += 1
        
        if self.counter % 250 == 0:
            self.get_logger().info(
                f"mag={mag_yaw_deg:.4f}, "
                f"gyro={math.degrees(gyro_yaw):.4f}, "
                f"error={math.degrees(yaw_error):.4f}, "
                f"fused={yaw_deg:.4f}"
            )
        #     self.get_logger().info(
        #         f"gyro_z_deg={gyro_z_deg:.2f}, "
        #         f"filtered={math.degrees(self.filtered_gyro_z):.2f}, "
        #         f"yaw_deg={yaw_deg:.2f}"
        #     )
        #     self.get_logger().info(
        #         f"ax={ax:.4f}, "
        #         f"ay={ay:.4f}, "
        #         f"az={az:.4f}"
        #         f"accel_pitch={accel_pitch:.6f}, "
        #         f"gyro_y={gyro_y:.6f}, "
        #         f"pith_deg={pitch_deg:.4f}"
        #     )
            # self.get_logger().info(
            #     f"gyro_yaw={math.degrees(gyro_yaw):.2f}, "
            #     f"mag_yaw_ref={self.mag_yaw_reference:.2f}, "
            #     f"mx={self.mag_x:.2f}, "
            #     f"my={self.mag_y:.2f}, "
            #     f"mag_yaw={math.degrees(mag_yaw):.2f}, "
            #     f"est_yaw={math.degrees(self.yaw):.2f}"
            # )
            # self.get_logger().info(
            #     f"accel x y z = {ax:.4f} {ay:.4f} {az:.4f}, "
            #     f"gyro roll pitch yaw = {msg.gyro_x:.4f} {msg.gyro_y:.4f} {msg.gyro_z:.4f}, "
            #     f"accel roll pitch = {accel_roll:.4f} {accel_pitch:.4f}, "
            #     f"current roll pitch yaw = {roll_deg:.4f} {pitch_deg:.4f} {yaw_deg:.4f}"
            # )


def main(args = None):
    rclpy.init(args=args)
    node = EstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

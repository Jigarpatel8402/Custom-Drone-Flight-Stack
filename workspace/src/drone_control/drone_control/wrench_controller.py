import rclpy
from rclpy.node import Node
from drone_model import Vehicle, Inertia
from drone_interfaces.msg import DroneCollective, DroneControl, DronePose, DroneWrench

class WrenchController(Node):
    def __init__(self):
        super().__init__('wrench_control')
        self.collective_force = 0.0
        self.roll_output = 0.0
        self.pitch_output = 0.0
        self.yaw_output = 0.0

        self.counter = 0
        
        self.declare_parameter('deadband', 0.001)
        self.deadband = self.get_parameter('deadband').value
        
        self.collective_sub = self.create_subscription(
            DroneCollective,
            '/control_collective',
            self.collective_callback,
            10
        )
        
        self.attitude_sub = self.create_subscription(
            DroneControl,
            '/control_rates',
            self.attitude_callback,
            10
        )

        # self.setpoint_sub = self.create_subscription(
        #     DronePose,
        #     '/drone_setpoint',
        #     self.setpoint_callback,
        #     10
        # )
        
        self.wrench_pub = self.create_publisher(
            DroneWrench,
            '/control_output',
            10
        )
        
        self.create_timer(
            0.004,
            self.control_loop
        )
        self.dt = 0.004
    
    def collective_callback(self, msg):
        self.collective_force = msg.force
    
    def attitude_callback(self, msg):
        self.roll_output = max(
            -1.0,
            min(1.0, msg.roll)
        )
        self.pitch_output = max(
            -1.0,
            min(1.0, msg.pitch)
        )
        self.yaw_output = max(
            -1.0,
            min(1.0, msg.yaw)
        )
    
    # def setpoint_callback(self, msg):
    #     self.roll_output = max(
    #         -1.0,
    #         min(1.0, msg.roll)
    #     )
    #     self.pitch_output = max(
    #         -1.0,
    #         min(1.0, msg.pitch)
    #     )
    #     self.yaw_output = max(
    #         -1.0,
    #         min(1.0, msg.yaw)
    #     )
    
    def body_torque(self, roll_output, pitch_output, yaw_output):
        return (
            roll_output * Inertia.MAX_ROLL_TORQUE,
            pitch_output * Inertia.MAX_PITCH_TORQUE,
            yaw_output * Inertia.MAX_YAW_TORQUE
        )
    
    def battery_compansation(self, thrust):     # need to modify later
        return thrust
    
    def control_loop(self):
        wrench_msg = DroneWrench()
        
        if abs(self.roll_output) < self.deadband:
            self.roll_output = 0.0
        if abs(self.pitch_output) < self.deadband:
            self.pitch_output = 0.0
        if abs(self.yaw_output) < self.deadband:
            self.yaw_output = 0.0
        
        roll_torque, pitch_torque, yaw_torque = self.body_torque(
            self.roll_output,
            self.pitch_output,
            self.yaw_output
        )

        wrench_msg.thrust = self.collective_force
        wrench_msg.roll_torque = roll_torque
        wrench_msg.pitch_torque = pitch_torque
        wrench_msg.yaw_torque = yaw_torque
        
        self.wrench_pub.publish(wrench_msg)

        self.counter += 1
        
        if self.counter % 100 == 0:
            self.get_logger().info(
                # f"desired_collective={self.collective_force:.4f}, "
                f"thrust={wrench_msg.thrust:.10f}, "
                # f"roll_output={self.roll_output:.4f}, "
                # f"max_roll_torque={Inertia.MAX_ROLL_TORQUE:.4f}, "
                f"roll_torque={wrench_msg.roll_torque:.4f}, "
                f"pitch_torque={wrench_msg.pitch_torque:.4f}, "
                f"yaw_torque={wrench_msg.yaw_torque:.4f}, "
                f"max_yaw_torque={Inertia.MAX_YAW_TORQUE}"
            )


def main(args = None):
    rclpy.init(args = args)
    node = WrenchController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

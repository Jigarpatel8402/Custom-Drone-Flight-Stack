import rclpy
from rclpy.node import Node
from drone_interfaces.msg import DroneWrench, DroneRotor
from drone_allocation.control_allocation import ControlAllocation

class AllocationNode(Node):
    def __init__(self):
        super().__init__('allocation_node')
        self.counter = 0
        self.control_command = None
        
        self.control_allocation = ControlAllocation()

        self.wrench_sub = self.create_subscription(
            DroneWrench,
            '/control_output',
            self.wrench_callback,
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
            'drone allocation started'
        )
    
    def wrench_callback(self, msg):
        self.control_command = msg
    
    def control_loop(self):
        if self.control_command is None:
            return
        
        motor_commands = self.control_allocation.allocate(self.control_command)
        
        allocation_msg = DroneRotor()
        allocation_msg.rotor0 = motor_commands[0]
        allocation_msg.rotor1 = motor_commands[1]
        allocation_msg.rotor2 = motor_commands[2]
        allocation_msg.rotor3 = motor_commands[3]
        
        self.motor_pub.publish(allocation_msg)

        self.counter += 1

        if self.counter % 100 == 0:
            self.get_logger().info(
                f"m1 = {motor_commands[0]:.4f}, "
                f"m2 = {motor_commands[1]:.4f}, "
                f"m3 = {motor_commands[2]:.4f}, "
                f"m4 = {motor_commands[3]:.4f}"
            )


def main(args = None):
    rclpy.init(args = args)
    node = AllocationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from drone_model.vehicle_model import Vehicle, ESC
from drone_model.modes import ArmState, ControlMode
from drone_interfaces.msg import DroneRotor, DroneMode
from drone_actuator.force_to_pwm import ForceTOPWM
from drone_actuator.motor_state import MotorState

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuator_node')
        self.counter = 0
        self.commands = None
        self.arm_state = ArmState.DISARMED
        self.control_mode = ControlMode.IDLE

        self.converter = ForceTOPWM()
        self.motors = [
            MotorState(0),
            MotorState(1),
            MotorState(2),
            MotorState(3)
        ]

        self.declare_parameter('timer', 0.0)               # need to change when uses in real drones
        self.timer = self.get_parameter('timer').value
        
        self.motor_sub = self.create_subscription(
            DroneRotor,
            '/motor_commands',
            self.motor_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            DroneMode,
            '/drone_mode',
            self.mode_callback,
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
        
        self.get_logger().info("drone actuator started")
    
    def motor_callback(self, msg):
        self.commands = [
            msg.rotor0,
            msg.rotor1,
            msg.rotor2,
            msg.rotor3
        ]
    
    def mode_callback(self, msg):
        self.arm_state = msg.arm_state
        self.control_mode = msg.control_mode
    
    def control_loop(self):
        output = list()
        
        if self.arm_state == ArmState.DISARMED:
            output = 4 * [ESC.MIN_PWM]
        elif self.arm_state == ArmState.ARMED and self.control_mode == ControlMode.IDLE:
            output = 4 * [Vehicle.ARM_PWM]
        else :
            if self.commands == None:
                return
            for motor, force in zip(self.motors, self.commands):
                motor.desired_force = force
                self.converter.update(motor)
                output.append(motor.pwm)
        
        pwm_msg = DroneRotor()
        pwm_msg.rotor0 = output[0]
        pwm_msg.rotor1 = output[1]
        pwm_msg.rotor2 = output[2]
        pwm_msg.rotor3 = output[3]

        self.pwm_pub.publish(pwm_msg)

        self.counter += 1
        
        if self.counter % 100 == 0:
            # self.get_logger().info(
            #     f"arm_status: {self.arm_state}, "
            #     f"control_mode: {self.control_mode}"
            # )
            self.get_logger().info(
                f"PWM: "
                f"ESC0={pwm_msg.rotor0:.1f}, "
                f"ESC1={pwm_msg.rotor1:.1f}, "
                f"ESC2={pwm_msg.rotor2:.1f}, "
                f"ESC3={pwm_msg.rotor3:.1f}"
            )


def main(args = None):
    rclpy.init(args = args)
    node = ActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

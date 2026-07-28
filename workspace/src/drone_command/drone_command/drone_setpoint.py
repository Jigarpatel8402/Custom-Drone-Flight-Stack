import rclpy
from rclpy.node import Node
import threading
from drone_model.constants import Physics
from drone_model.modes import ArmState, ControlMode
from drone_command.mode_manager import ModeManager
from drone_interfaces.msg import DronePose, DroneMode

class CommandsPublisher(Node):
    def __init__(self):
        super().__init__('drone_setpoint')
        self.desired_throttle = 0.0
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0
        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0

        self.mode = ModeManager()
        
        self.drone_cmd_pub = self.create_publisher(
            DronePose,
            '/drone_setpoint',
            10
        )

        self.drone_mode_pub = self.create_publisher(
            DroneMode,
            '/drone_mode',
            10
        )

        self.pub_timer = self.create_timer(
            0.05,
            self.pub_callback
        )

    def pub_callback(self):
        cmd_msg = DronePose()
        mode_msg = DroneMode()

        cmd_msg.throttle = self.desired_throttle
        cmd_msg.roll = Physics.DEG2RAD * self.desired_roll
        cmd_msg.pitch = Physics.DEG2RAD * self.desired_pitch
        cmd_msg.yaw = Physics.DEG2RAD * self.desired_yaw
        cmd_msg.x = self.desired_x
        cmd_msg.y = self.desired_y
        cmd_msg.z = self.desired_z

        mode_msg.arm_state = self.mode.arm_mode
        mode_msg.control_mode = self.mode.control_mode

        self.drone_cmd_pub.publish(cmd_msg)
        self.drone_mode_pub.publish(mode_msg)


class CommandsControl():
    def __init__(self, cmdpub: CommandsPublisher):
        self.cmdpub = cmdpub
        self.exit_event = threading.Event()

        threading.Thread(
            target = self.input_call,
            daemon = True
        ).start()

        self.COMMANDS = {
            "exit": self.exit,
            "arm": self.arm,
            "disarm": self.disarm,
            "manual": self.manual_mode,
            "hover": self.hover_mode,
            "throttle": self.set_throttle,
            "attitude": self.set_attitude,
            "takeoff": self.takeoff,
            "position": self.set_position
        }
        
    def input_call(self):
        while not self.exit_event.is_set():
            cmd = input(">> ").split()

            if len(cmd) == 0:
                continue
            
            if cmd[0] in self.COMMANDS:
                if len(cmd) == 1:
                    self.COMMANDS[cmd[0]]()
                elif len(cmd) >= 2 and self.cmdpub.mode.arm_mode == ArmState.ARMED:
                    self.COMMANDS[cmd[0]](cmd[1:])
                elif len(cmd) >= 2 and self.cmdpub.mode.arm_mode == ArmState.DISARMED:
                    print("Drone is DISARMED")
                    continue
            else :
                print("Command NOT Found")
                continue
    
    def exit(self):
        self.exit_event.set()
        print("Setpoint closed")
        return
    
    def arm(self):
        self.cmdpub.mode.arm_mode = ArmState.ARMED
        self.cmdpub.mode.control_mode = ControlMode.IDLE
        print(f"Drone Armed")
    
    def disarm(self):
        self.cmdpub.desired_throttle = 0.0
        self.cmdpub.desired_roll = 0.0
        self.cmdpub.desired_pitch = 0.0
        self.cmdpub.desired_yaw = 0.0
        self.cmdpub.desired_x = 0.0
        self.cmdpub.desired_y = 0.0
        self.cmdpub.desired_z = 0.0
        self.cmdpub.mode.arm_mode = ArmState.DISARMED
        print(f"Drone disarmed")
    
    def manual_mode(self):
        self.cmdpub.mode.control_mode = ControlMode.MANUAL
    
    def hover_mode(self):
        self.cmdpub.mode.control_mode = ControlMode.HOVER
        print("Drone Set at Hover")
    
    def takeoff_mode(self):
        self.cmdpub.mode.control_mode = ControlMode.ALTITUDE
    
    def position_mode(self):
        self.cmdpub.mode.control_mode = ControlMode.POSITION
    
    def set_throttle(self, throttle = None):
        self.manual_mode()
        if throttle is None or len(throttle) != 1:
            print("throttle <value>")
            return
        try :
            self.cmdpub.desired_throttle = float(throttle[0])
            print(f"throttle set at {self.cmdpub.desired_throttle}")
        except ValueError:
            print("Invalid Throttle value")
            return
    
    def set_attitude(self, attitude = None):        # make it only accept yaw
        self.manual_mode()
        if attitude is None or len(attitude) != 3:
            print("attitude <roll pitch yaw>")
            return
        try:
            roll = float(attitude[0])
            pitch = float(attitude[1])
            yaw = float(attitude[2])
        except ValueError:
            print("Invalid attitude value")
            return
        
        if -360.0 <= roll <= 360.0 and -360.0 <= pitch <= 360.0 and -360.0 <= yaw <= 360.0:
            self.cmdpub.desired_roll = roll
            self.cmdpub.desired_pitch = pitch
            self.cmdpub.desired_yaw = yaw
            print(f"Attitude set at {self.cmdpub.desired_roll} {self.cmdpub.desired_pitch} {self.cmdpub.desired_yaw}")
        else :
            print(f"Attitude value must be in between -360.0 to +360.0")
    
    def takeoff(self, altitude = None):
        self.takeoff_mode()
        if altitude == None or len(altitude) != 1:
            print("takeoff <z>")
            return
        self.set_position([0.0, 0.0] + altitude)
    
    def set_position(self, position = None):
        self.position_mode()
        if position is None or len(position) != 3:
            print("position <x y z>")
            return
        try:
            x = float(position[0])
            y = float(position[1])
            z = float(position[2])
        except ValueError:
            print("Invalid position value")
            return
        
        self.cmdpub.desired_x = x
        self.cmdpub.desired_y = y
        self.cmdpub.desired_z = z
        print(f"position set at {self.cmdpub.desired_x} {self.cmdpub.desired_y} {self.cmdpub.desired_z}")


def main(args = None):
    rclpy.init(args = args)
    node = CommandsPublisher()
    command = CommandsControl(node)
    while rclpy.ok() and not command.exit_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

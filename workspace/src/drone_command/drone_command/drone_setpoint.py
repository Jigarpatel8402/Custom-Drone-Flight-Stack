import rclpy
from rclpy.node import Node
import threading
from drone_interfaces.msg import DronePose

# MODE_DISARMED = 0
# MODE_ARMED = 1
# MODE_ATTITUDE = 2
# MODE_ALTITUDE = 3
# MODE_POSITION = 4
# MODE_LAND = 5

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

        self.declare_parameter('arm_throttle', 100.0)
        self.arm_throttle = self.get_parameter('arm_throttle').value

        self.drone_cmd_pub = self.create_publisher(
            DronePose,
            '/drone_setpoint',
            10
        )

        self.pub_timer = self.create_timer(
            0.05,
            self.pub_callback
        )

    def pub_callback(self):
        cmd_msg = DronePose()
        
        cmd_msg.throttle = self.desired_throttle
        cmd_msg.roll = self.desired_roll
        cmd_msg.pitch = self.desired_pitch
        cmd_msg.yaw = self.desired_yaw
        cmd_msg.x = self.desired_x
        cmd_msg.y = self.desired_y
        cmd_msg.z = self.desired_z

        self.drone_cmd_pub.publish(cmd_msg)


class CommandsControl():
    def __init__(self, cmdpub: CommandsPublisher):
        self.cmdpub = cmdpub
        self.running = True

        threading.Thread(
            target = self.input_call,
            daemon = True
        ).start()
        
    def input_call(self):               # add logical simple method insted of if elif
        while self.running:
            cmd = input(">> ").split()

            if len(cmd) < 2:
                continue

            if f"{cmd[0]} {cmd[1]}" == 'arm throttle':
                self.arm()
            elif f"{cmd[0]} {cmd[1]}" == 'set throttle':
                if len(cmd) != 3:
                    print("set throttle <value>")
                self.set_throttle(cmd[2])
            elif f"{cmd[0]} {cmd[1]}" == 'set attitude':
                if len(cmd) != 5:
                    print("set attitude <roll pitch yaw>")
                    continue
                self.set_attitude(cmd[2], cmd[3], cmd[4])
            elif f"{cmd[0]} {cmd[1]}" == 'set position':
                if len(cmd) != 5:
                    print("set position <x y z>")
                    continue
                self.set_position(cmd[2], cmd[3], cmd[4])
            else :
                print("Command NOT found.")
    
    def arm(self):
        self.cmdpub.desired_throttle = float(self.cmdpub.arm_throttle)
        print(f"Throttle armed at {self.cmdpub.arm_throttle}")
    
    def set_throttle(self, throttle):
        self.cmdpub.desired_throttle = float(throttle)
        print(f"throttle set at {self.cmdpub.desired_throttle}")
    
    def set_attitude(self, roll, pitch, yaw):       # make it only accept yaw
        if -360.0 <= float(roll) <= 360.0 and -360.0 <= float(pitch) <= 360.0 and -360.0 <= float(yaw) <= 360.0:
            self.cmdpub.desired_roll = float(roll)
            self.cmdpub.desired_pitch = float(pitch)
            self.cmdpub.desired_yaw = float(yaw)
            print(f"Attitude set at {self.cmdpub.desired_roll} {self.cmdpub.desired_pitch} {self.cmdpub.desired_yaw}")
        else :
            print(f"Attitude value must be in between -360.0 to +360.0")
    
    def set_position(self, x, y, z):
        self.cmdpub.desired_x = float(x)
        self.cmdpub.desired_y = float(y)
        self.cmdpub.desired_z = float(z)
        print(f"position set at {self.cmdpub.desired_x} {self.cmdpub.desired_y} {self.cmdpub.desired_z}")


def main(args = None):
    rclpy.init(args = args)
    node = CommandsPublisher()
    command = CommandsControl(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

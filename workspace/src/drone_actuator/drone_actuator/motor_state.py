class MotorState:
    def __init__(self, motor_id: int):
        self.id = motor_id

        self.desired_force = 0.0
        self.current_force = 0.0
        
        self.pwm = 1000.0
        self.throttle = 0.0

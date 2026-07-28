from drone_model.modes import ArmState, ControlMode

class ModeManager:
    def __init__(self):
        self.arm_mode = ArmState.DISARMED
        self.control_mode = ControlMode.IDLE

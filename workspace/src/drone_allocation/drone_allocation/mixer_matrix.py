from drone_model import Frame, Propellers
from drone_allocation.allocation_matrix import AllocationMatrix

class MixerMatrix:
    def __init__(self):
        self.motors = Frame.MOTORS
        self.arm_length = Frame.ARM_LENGTH

        self.allocation = self.build()

    def build(self):
        allocation = AllocationMatrix()

        for motor in self.motors:
            allocation.matrix.append([
                0.25,
                motor.y / self.arm_length,
                -motor.x / self.arm_length,
                -motor.direction * (Propellers.KM / Propellers.KF)
            ])
        
        return allocation

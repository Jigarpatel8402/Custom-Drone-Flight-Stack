from drone_model import Vehicle, Frame, Propellers
from drone_allocation.allocation_matrix import AllocationMatrix
from drone_allocation.saturation import Saturation

class ControlAllocation:
    def __init__(self):
        self.frame = Frame()
        self.propellers = Propellers()
        self.allocation = AllocationMatrix(self.frame, self.propellers)
    
    def allocate(self, commands):
        wrench = [
            commands.thrust,
            commands.roll_torque,
            commands.pitch_torque,
            commands.yaw_torque
        ]

        forces = self.allocation.allocate(wrench)
        
        return Saturation.apply(
            forces, 
            0.0, 
            Vehicle.MAX_THRUST_PER_MOTOR
        )

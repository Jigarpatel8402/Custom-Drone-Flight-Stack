import numpy as np

class AllocationMatrix:
    def __init__(self, frame, propellers):
        self.frame = frame
        self.propellers = propellers
        self.motors = self.frame.MOTORS
        self.arm_length = self.frame.ARM_LENGTH

        self.A = self.build()
        self.A_inv = self.inv_A()
    
    def build(self):
        rows = [
            [],     # Total thust
            [],     # Roll torque
            [],     # Pitch torque
            []      # Yaw torque
        ]
        
        for motor in self.motors:
            rows[0].append(1.0)
            rows[1].append(motor.y / self.arm_length)
            rows[2].append(-motor.x / self.arm_length)
            rows[3].append(
                -motor.direction * (self.propellers.KM / self.propellers.KF)
            )
        
        return np.array(rows)
    
    def inv_A(self):
        if np.linalg.det(self.A) == 0:
            raise RuntimeError("Allocation matrix is singular")
        else :
            return np.linalg.inv(self.A)
    
    def allocate(self, wrench):
        wrench = np.array(wrench)
        
        forces = self.A_inv @ wrench
        
        return forces.tolist()

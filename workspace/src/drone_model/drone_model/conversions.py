import math
from drone_model.constants import Physics
from drone_model.vehicle_model import ESC

class Conversion:

    @staticmethod
    def pwm_to_throttle(pwm):
        return (
            pwm - ESC.MIN_PWM
        ) / (
            ESC.MAX_PWM - ESC.MIN_PWM
        )
    
    @staticmethod
    def throttle_to_pwm(throttle):
        return (
            ESC.MIN_PWM +
            throttle * (ESC.MAX_PWM - ESC.MIN_PWM)
        )
    
    @staticmethod
    def omega_to_rpm(omega):
        return omega * 60.0 / (2.0 * math.pi)
    
    @staticmethod
    def rpm_to_omega(rpm):
        return rpm * 2.0 * math.pi / 60.0

from math import sqrt
from drone_model.vehicle_model import ESC, Motors, Propellers

class ForceTOPWM:
    def update(self, motor):
        force = max(
            0.0,
            motor.desired_force
        )
        omega = sqrt(force / Propellers.KF)
        
        motor.throttle = (omega / Motors.MAX_OMEGA)
        motor.throttle = max(
            ESC.THROTTLE_MIN,
            min(ESC.THROTTLE_MAX, motor.throttle)
        )

        motor.pwm = (
            ESC.MIN_PWM + 
            motor.throttle * (ESC.MAX_PWM - ESC.MIN_PWM)
        )

#include "drone_gazebo_plugins/rotor_physics.hpp"
#include <algorithm>

RotorPhysics::RotorPhysics()
{
    minPWM = 1000.0;
    maxPWM = 2000.0;

    maxOmega = 900.0;

    tauUp = 0.012;
    tauDown = 0.025;

    KF = 8.54858e-6;
    KM = 1.6e-7;
}

void RotorPhysics::Update(
    RotorState &motor,
    double dt
)
{
    // ESC Model
    motor.throttle = (
        (motor.pwm - minPWM) /
        (maxPWM - minPWM)
    );

    motor.throttle = std::clamp(
            motor.throttle,
            0.0,
            1.0
        );
    
    motor.desiredOmega = (
        motor.throttle *
        maxOmega
    );
    
    // Motor Model
    double error = (
        motor.desiredOmega - 
        motor.currentOmega
    );

    double tau = error >= 0 ? tauUp : tauDown;
    
    motor.omegaDot = error / tau;

    motor.currentOmega += motor.omegaDot * dt;

    motor.currentOmega = std::clamp(
        motor.currentOmega,
        0.0,
        900.0
    );
    
    // Propeller Model
    double omega2 = motor.currentOmega * motor.currentOmega;

    motor.thrust = KF * omega2;

    motor.reactiveTorque = motor.direction * KM * omega2;
}

#pragma once

struct RotorState
{
    int id;
    double direction;      // +1 = CW, -1 = CCW
    
    double pwm = 1000.0;
    double throttle = 0.0;

    double desiredOmega = 0.0;
    double currentOmega = 0.0;
    double omegaDot = 0.0;

    double thrust = 0.0;
    double reactiveTorque = 0.0;

    RotorState() = default;

    RotorState(int id_, int dir)
    {
        id = id_;
        direction = dir;
    }
};

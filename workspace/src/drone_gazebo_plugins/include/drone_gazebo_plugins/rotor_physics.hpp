#pragma once

#include "rotor_state.hpp"

class RotorPhysics
{
    public:
        RotorPhysics();

        void Update(
            RotorState &motor,
            double dt
        );
    
    private:
        double minPWM;
        double maxPWM;

        double maxOmega;

        double tauUp;
        double tauDown;

        double KF;
        double KM;
};

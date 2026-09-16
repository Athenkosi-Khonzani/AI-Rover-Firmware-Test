#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include <Arduino.h>
#include <Motor.h>

// Vehicle-level directions supported by the standard four-wheel drivetrain.
#define Clockwise     4
#define Contrarotate  5

class vehicle
{
    public:
        void Init();
        void Move(int Dir, int Speed);
        void StopVehicle();

    private:
        motor _motor;
};

#endif

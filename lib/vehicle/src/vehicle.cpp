#include "vehicle.h"
#include "Arduino.h"

void vehicle::Init()
{
    _motor.Init();
}

void vehicle::Move(int Dir, int Speed)
{
    Speed = constrain(Speed, 0, 255);

    switch (Dir)
    {
        case Forward:
            _motor.Motor(Forward, Speed);
            break;

        case Backward:
            _motor.Motor(Backward, Speed);
            break;

        case Clockwise:
            // Left wheels forward, right wheels backward.
            _motor.Motor(MOTOR_FL, Forward, Speed);
            _motor.Motor(MOTOR_RL, Forward, Speed);
            _motor.Motor(MOTOR_FR, Backward, Speed);
            _motor.Motor(MOTOR_RR, Backward, Speed);
            break;

        case Contrarotate:
            // Left wheels backward, right wheels forward.
            _motor.Motor(MOTOR_FL, Backward, Speed);
            _motor.Motor(MOTOR_RL, Backward, Speed);
            _motor.Motor(MOTOR_FR, Forward, Speed);
            _motor.Motor(MOTOR_RR, Forward, Speed);
            break;

        case Stop:
        default:
            _motor.StopAll();
            break;
    }
}

void vehicle::StopVehicle()
{
    _motor.StopAll();
}

#include "Motor.h"
#include "Arduino.h"

void motor::Init()
{
    // Keep the drivers asleep while GPIO directions are being configured.
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    pinMode(FL_IN1_PIN, OUTPUT);
    pinMode(FL_IN2_PIN, OUTPUT);
    pinMode(FR_IN1_PIN, OUTPUT);
    pinMode(FR_IN2_PIN, OUTPUT);
    pinMode(RL_IN1_PIN, OUTPUT);
    pinMode(RL_IN2_PIN, OUTPUT);
    pinMode(RR_IN1_PIN, OUTPUT);
    pinMode(RR_IN2_PIN, OUTPUT);

    // Start every H-bridge in coast/stop.
    analogWrite(FL_IN1_PIN, 0);
    analogWrite(FL_IN2_PIN, 0);
    analogWrite(FR_IN1_PIN, 0);
    analogWrite(FR_IN2_PIN, 0);
    analogWrite(RL_IN1_PIN, 0);
    analogWrite(RL_IN2_PIN, 0);
    analogWrite(RR_IN1_PIN, 0);
    analogWrite(RR_IN2_PIN, 0);

    Enable(true);
}

void motor::_74HC595D()
{
    // The new PCB no longer uses a 74HC595. This function remains only for
    // compatibility with older project code.
    Init();
}

void motor::Enable(bool enable)
{
    digitalWrite(MOTOR_ENABLE_PIN, enable ? HIGH : LOW);

    // Give the DRV8833 a short wake-up time after nSLEEP is asserted HIGH.
    if (enable)
    {
        delay(1);
    }
}

void motor::DrivePair(uint8_t in1Pin, uint8_t in2Pin, int dir, int speed)
{
    speed = constrain(speed, 0, 255);

    switch (dir)
    {
        case Forward:
            // DRV8833: IN1 PWM, IN2 LOW
            analogWrite(in1Pin, speed);
            analogWrite(in2Pin, 0);
            break;

        case Backward:
            // DRV8833: IN1 LOW, IN2 PWM
            analogWrite(in1Pin, 0);
            analogWrite(in2Pin, speed);
            break;

        case Brake:
            // Both inputs HIGH = brake.
            analogWrite(in1Pin, 255);
            analogWrite(in2Pin, 255);
            break;

        case Stop:
        default:
            // Both inputs LOW = coast/stop.
            analogWrite(in1Pin, 0);
            analogWrite(in2Pin, 0);
            break;
    }
}

void motor::Motor(uint8_t motorID, int dir, int speed)
{
    switch (motorID)
    {
        case MOTOR_FL:
            DrivePair(FL_IN1_PIN, FL_IN2_PIN, dir, speed);
            break;

        case MOTOR_FR:
            DrivePair(FR_IN1_PIN, FR_IN2_PIN, dir, speed);
            break;

        case MOTOR_RL:
            DrivePair(RL_IN1_PIN, RL_IN2_PIN, dir, speed);
            break;

        case MOTOR_RR:
            DrivePair(RR_IN1_PIN, RR_IN2_PIN, dir, speed);
            break;

        default:
            break;
    }
}

void motor::Motor(int dir, int speed)
{
    Motor(MOTOR_FL, dir, speed);
    Motor(MOTOR_FR, dir, speed);
    Motor(MOTOR_RL, dir, speed);
    Motor(MOTOR_RR, dir, speed);
}

void motor::StopAll()
{
    Motor(Stop, 0);
}

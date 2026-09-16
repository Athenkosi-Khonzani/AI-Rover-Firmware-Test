#include "Motor.h"
#include "Arduino.h"
#include <math.h>

// 2S pack sanity limits. Motion is refused when the measured voltage is
// outside this range because the PWM mapping would no longer be trustworthy.
static const float MIN_VALID_BATTERY_VOLTAGE = 6.0f;
static const float MAX_VALID_BATTERY_VOLTAGE = 8.6f;

// Battery voltage changes slowly relative to the wheel-speed control loop.
// Caching prevents every motor command from blocking on ADC averaging.
static const unsigned long BATTERY_REFRESH_INTERVAL_MS = 500;

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

    _battery.Init();
    _batteryValid = false;
    _batteryVoltage = 0.0f;
    _minimumPWM = 0;
    _maximumPWM = 0;
    _lastBatteryReadMs = 0;

    // Use a more heavily averaged first reading. Move() will refuse motion if
    // this reading is absent or outside the valid 2S battery range.
    RefreshBatteryVoltage(32);

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

int motor::CorrectDirection(uint8_t motorID, int dir) const
{
    // The two left motors are mounted as mirror images of the right motors.
    // Reverse only the logical motion directions; Stop and Brake are unchanged.
    if (motorID == MOTOR_FL || motorID == MOTOR_RL)
    {
        if (dir == Forward)
        {
            return Backward;
        }

        if (dir == Backward)
        {
            return Forward;
        }
    }

    return dir;
}

bool motor::SetMotorVoltageRange(float minimumVolts, float maximumVolts)
{
    if (!isfinite(minimumVolts) || !isfinite(maximumVolts) ||
        minimumVolts <= 0.0f || maximumVolts < minimumVolts)
    {
        return false;
    }

    _motorMinimumVoltage = minimumVolts;
    _motorMaximumVoltage = maximumVolts;

    if (_batteryValid)
    {
        return CalculatePWMLimits();
    }

    return true;
}

bool motor::CalculatePWMLimits()
{
    if (!isfinite(_batteryVoltage) ||
        _batteryVoltage < MIN_VALID_BATTERY_VOLTAGE ||
        _batteryVoltage > MAX_VALID_BATTERY_VOLTAGE)
    {
        _minimumPWM = 0;
        _maximumPWM = 0;
        _batteryValid = false;
        return false;
    }

    // Round the minimum upward so a nonzero command reaches at least the
    // configured minimum motor voltage. Round the maximum downward so the
    // average applied voltage does not exceed the configured maximum.
    _minimumPWM = static_cast<int>(ceilf(
        (_motorMinimumVoltage / _batteryVoltage) * 255.0f));
    _maximumPWM = static_cast<int>(floorf(
        (_motorMaximumVoltage / _batteryVoltage) * 255.0f));

    _minimumPWM = constrain(_minimumPWM, 1, 255);
    _maximumPWM = constrain(_maximumPWM, 1, 255);

    if (_minimumPWM > _maximumPWM)
    {
        _minimumPWM = 0;
        _maximumPWM = 0;
        _batteryValid = false;
        return false;
    }

    _batteryValid = true;
    return true;
}

bool motor::RefreshBatteryVoltage(uint8_t samples)
{
    _batteryVoltage = _battery.ReadVoltage(samples);
    _lastBatteryReadMs = millis();
    return CalculatePWMLimits();
}

void motor::RefreshBatteryIfDue()
{
    if (!_batteryValid ||
        (millis() - _lastBatteryReadMs >= BATTERY_REFRESH_INTERVAL_MS))
    {
        RefreshBatteryVoltage(8);
    }
}

int motor::SpeedPercentToPWM(float speedPercent) const
{
    if (!_batteryValid || speedPercent <= 0.0f)
    {
        return 0;
    }

    speedPercent = constrain(speedPercent, 1.0f, 100.0f);

    // Reserve 0% for stop. Map 1% exactly to minimum PWM and 100% exactly to
    // maximum PWM so every nonzero command lies in the runnable voltage range.
    float fraction = (speedPercent - 1.0f) / 99.0f;
    float pwm = _minimumPWM + fraction * (_maximumPWM - _minimumPWM);

    return constrain(static_cast<int>(roundf(pwm)),
                     _minimumPWM,
                     _maximumPWM);
}

bool motor::Move(uint8_t motorID, int dir, float speedPercent)
{
    // Stop and Brake must remain immediately available even if battery sensing
    // has failed. A zero-speed motion request is treated as Stop.
    if (dir == Stop || speedPercent <= 0.0f)
    {
        Motor(motorID, Stop, 0);
        return true;
    }

    if (dir == Brake)
    {
        Motor(motorID, Brake, 255);
        return true;
    }

    RefreshBatteryIfDue();

    if (!_batteryValid)
    {
        // A shared supply fault affects the whole drivetrain, not only the
        // motor addressed by this command.
        StopAll();
        return false;
    }

    int pwm = SpeedPercentToPWM(speedPercent);
    Motor(motorID, dir, pwm);
    return true;
}

bool motor::Move(int dir, float speedPercent)
{
    // Refresh and map once so every wheel receives the same PWM for this call.
    if (dir == Stop || speedPercent <= 0.0f)
    {
        StopAll();
        return true;
    }

    if (dir == Brake)
    {
        Motor(Brake, 255);
        return true;
    }

    RefreshBatteryIfDue();

    if (!_batteryValid)
    {
        StopAll();
        return false;
    }

    int pwm = SpeedPercentToPWM(speedPercent);
    Motor(MOTOR_FL, dir, pwm);
    Motor(MOTOR_FR, dir, pwm);
    Motor(MOTOR_RL, dir, pwm);
    Motor(MOTOR_RR, dir, pwm);
    return true;
}

float motor::GetBatteryVoltage() const
{
    return _batteryVoltage;
}

int motor::GetMinimumPWM() const
{
    return _minimumPWM;
}

int motor::GetMaximumPWM() const
{
    return _maximumPWM;
}

void motor::Motor(uint8_t motorID, int dir, int speed)
{
    int correctedDirection = CorrectDirection(motorID, dir);

    switch (motorID)
    {
        case MOTOR_FL:
            DrivePair(FL_IN1_PIN, FL_IN2_PIN, correctedDirection, speed);
            break;

        case MOTOR_FR:
            DrivePair(FR_IN1_PIN, FR_IN2_PIN, correctedDirection, speed);
            break;

        case MOTOR_RL:
            DrivePair(RL_IN1_PIN, RL_IN2_PIN, correctedDirection, speed);
            break;

        case MOTOR_RR:
            DrivePair(RR_IN1_PIN, RR_IN2_PIN, correctedDirection, speed);
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

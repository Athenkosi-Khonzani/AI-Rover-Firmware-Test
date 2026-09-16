#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <Arduino.h>
#include "Battery.h"

// -----------------------------------------------------------------------------
// ACEBOTT ESP32 Max V1.0 + Custom DRV8833 Shield V2
// Motor-control pin mapping from the PCB.
// -----------------------------------------------------------------------------
#define MOTOR_ENABLE_PIN 4   // Shared DRV8833 nSLEEP (HIGH = enabled)

#define FL_IN1_PIN 16
#define FL_IN2_PIN 33

#define FR_IN1_PIN 17
#define FR_IN2_PIN 23

#define RL_IN1_PIN 18
#define RL_IN2_PIN 26

#define RR_IN1_PIN 19
#define RR_IN2_PIN 27

// Motor identifiers.
#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_RL 3
#define MOTOR_RR 4

// Direction constants. These names preserve the style used by the old library.
#define Stop      0
#define Forward   1
#define Backward  2
#define Brake     3

class motor
{
    public:
        void Init();

        // Legacy alias retained so older sketches that called _74HC595D()
        // still initialise the new DRV8833-based shield correctly.
        void _74HC595D();

        void Enable(bool enable);

        // Preferred interface: drive one motor using a 0-100% speed command.
        //   0%      = stop
        //   1%      = calculated minimum runnable PWM
        //   100%    = calculated maximum safe PWM
        // The PWM limits are derived from battery voltage and the configured
        // motor-voltage range. Returns false if the battery reading is invalid.
        bool Move(uint8_t motorID, int dir, float speedPercent);

        // Percentage-based overload for all four motors.
        bool Move(int dir, float speedPercent);

        // Legacy raw-PWM interface retained for existing test sketches.
        // speed: 0-255. Prefer Move() in application and control-loop code.
        void Motor(uint8_t motorID, int dir, int speed);

        // Legacy raw-PWM overload for all four motors.
        void Motor(int dir, int speed);

        void StopAll();

        // Default range is 3.0-6.0 V. This can be changed later if the exact
        // TT-motor specification is confirmed to be different.
        bool SetMotorVoltageRange(float minimumVolts, float maximumVolts);

        // Force a new battery measurement. Normal Move() calls refresh the
        // cached value automatically at a limited rate.
        bool RefreshBatteryVoltage(uint8_t samples = 8);

        float GetBatteryVoltage() const;
        int GetMinimumPWM() const;
        int GetMaximumPWM() const;

    private:
        void DrivePair(uint8_t in1Pin, uint8_t in2Pin, int dir, int speed);
        int CorrectDirection(uint8_t motorID, int dir) const;
        int SpeedPercentToPWM(float speedPercent) const;
        bool CalculatePWMLimits();
        void RefreshBatteryIfDue();

        battery _battery;

        float _motorMinimumVoltage = 3.0f;
        float _motorMaximumVoltage = 6.0f;
        float _batteryVoltage = 0.0f;

        int _minimumPWM = 0;
        int _maximumPWM = 0;

        unsigned long _lastBatteryReadMs = 0;
        bool _batteryValid = false;
};

#endif

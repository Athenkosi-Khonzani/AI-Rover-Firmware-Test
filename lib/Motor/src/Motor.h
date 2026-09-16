#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <Arduino.h>

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

        // Drive one motor independently.
        // speed: 0-255
        void Motor(uint8_t motorID, int dir, int speed);

        // Backwards-compatible overload: drive all four motors together.
        void Motor(int dir, int speed);

        void StopAll();

    private:
        void DrivePair(uint8_t in1Pin, uint8_t in2Pin, int dir, int speed);
};

#endif

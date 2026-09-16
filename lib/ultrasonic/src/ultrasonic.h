#ifndef _ULTRASONIC_H__
#define _ULTRASONIC_H__

#include <Arduino.h>

// PCB pin mapping.
#define ULTRASONIC_TRIG_PIN 13
#define ULTRASONIC_ECHO_PIN 14

class ultrasonic
{
    public:
        // Initialise using the PCB's fixed ultrasonic pins.
        void Init();

        // Optional overload retained for flexibility/testing.
        void Init(int trigPin, int echoPin);

        // Returns distance in centimetres.
        // Returns -1.0 if no echo is received before timeout.
        float Ranging(unsigned long timeoutUs = 30000UL);

    private:
        int _trigPin = ULTRASONIC_TRIG_PIN;
        int _echoPin = ULTRASONIC_ECHO_PIN;
};

#endif

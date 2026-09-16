#include "ultrasonic.h"
#include "Arduino.h"

void ultrasonic::Init()
{
    Init(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
}

void ultrasonic::Init(int trigPin, int echoPin)
{
    _trigPin = trigPin;
    _echoPin = echoPin;

    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_trigPin, LOW);
}

float ultrasonic::Ranging(unsigned long timeoutUs)
{
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    unsigned long duration = pulseIn(_echoPin, HIGH, timeoutUs);

    if (duration == 0)
    {
        return -1.0f;
    }

    // Standard HC-SR04-style conversion: microseconds / 58 ~= centimetres.
    return static_cast<float>(duration) / 58.0f;
}

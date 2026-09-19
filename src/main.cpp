#include <Arduino.h>
#include <StraightDrive.h>

void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for Serial to be ready.
    goStraight(1000.0f);  // millimetres; returns after stopping
}

void loop() {}
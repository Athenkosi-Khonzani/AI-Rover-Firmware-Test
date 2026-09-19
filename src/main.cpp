#include <Arduino.h>
#include <StraightDrive.h>
#include <TurnDrive.h>

void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for Serial to be ready.
    //goStraight(2000.0f);  // millimetres; returns after stopping
    //turnLeft(90.0f);  // degrees; returns after stopping
    goStraight(1000.0f);
    delay(200);
    turnRight(90.0f);  // degrees; returns after stopping

    goStraight(1000.0f);
    delay(200);
    turnRight(90.0f);  // degrees; returns after stopping

    goStraight(1000.0f);
    delay(200);
    turnRight(90.0f);  // degrees; returns after stopping

    goStraight(1000.0f);
}

void loop() {}
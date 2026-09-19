#ifndef STRAIGHT_DRIVE_H
#define STRAIGHT_DRIVE_H

// Call Serial.begin(115200) in setup() if you want diagnostics.
// Optional: initialise motor and encoder hardware early. goStraight() also
// initialises automatically on its first call.
bool beginStraightDrive();

// Blocking: drive forward for the requested encoder-estimated distance (mm).
// Returns false for an invalid distance, invalid battery reading, a wheel
// without encoder ticks, a motor-command failure, or the overall timeout.
// On return, all motors are stopped and the driver is put to sleep.
bool goStraight(float distanceMM);

#endif

#ifndef _TURN_DRIVE_H_
#define _TURN_DRIVE_H_

// In-place turn primitive for the AI Learner Rover.
// Turns the rover on the spot by a given angle, using encoder feedback.
// Positive angle = magnitude in degrees; direction chosen by which call.
//
// Usage:
//   turnLeft(90);    // spin 90 degrees left (counter-clockwise)
//   turnRight(90);   // spin 90 degrees right (clockwise)
//   turnLeft(180);   // about-face
//
// TRACK_WIDTH_MM in TurnDrive.cpp is the calibration constant: command 90,
// measure the actual turn, then W_new = W_old * (requested / actual).

bool turnLeft(float angleDeg);
bool turnRight(float angleDeg);

#endif

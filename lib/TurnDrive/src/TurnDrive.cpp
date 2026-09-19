#include "TurnDrive.h"
#include <Arduino.h>
#include <Motor.h>
#include <encoder.h>
#include <math.h>

namespace {
motor Motor;
encoder Encoders;
bool initialized = false;

// ---- Wheel / encoder geometry (must match StraightDrive) ----
static const float WHEEL_DIAMETER_MM = 60.0f;

// Wheel order: front left, front right, rear left, rear right.
static const uint8_t SLOT_COUNTS[4] = {16, 20, 16, 16};

// Set to 1 for RISING-only counting, 2 if the library counts both edges.
// KEEP THIS THE SAME AS StraightDrive.
static const uint8_t COUNTED_EDGES_PER_SLOT = 2;

static const float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;

// ---- THE turn calibration constants (one per direction) ----
// Effective track width = the value that makes the turn come out right, NOT
// the physical width: Mecanum wheels scrub sideways, and left vs right spins
// often scrub differently, so each direction is calibrated separately.
// Calibrate each: command 90 deg in that direction, measure the ACTUAL angle,
// then scale:  W_new = W_old * (commanded_deg / actual_deg).
static const float TRACK_WIDTH_LEFT_MM  = 310.0f;   // For whowever touches this code after me: CALIBRATE (left turns)
static const float TRACK_WIDTH_RIGHT_MM = 250.0f;   // <-- CALIBRATE (right turns)

// ---- Speeds (slow for calibration, same spirit as StraightDrive) ----
static const float CRUISE_SPEED_PERCENT = 5.0f;
static const float APPROACH_SPEED_PERCENT = 2.0f;
static const float MIN_ACTIVE_SPEED_PERCENT = 1.0f;
static const float MAX_SPEED_PERCENT = 30.0f;
// Start easing off when the remaining arc is within this many mm of wheel travel.
static const float SLOWDOWN_ARC_MM = 40.0f;

// ---- Balance correction: keep the two sides turning at equal arc rates ----
// If one side out-runs the other the centre of rotation drifts, so nudge the
// faster side down. Positive balanceError means the left side has travelled
// further than the right.
static const float BALANCE_KP = 0.10f;           // command % per mm of side error
static const float MAX_BALANCE_CORRECTION_PERCENT = 5.0f;
// Front/rear evenness within one side (same idea as StraightDrive).
static const float SAME_SIDE_KP = 0.085f;
static const float MAX_SAME_SIDE_CORRECTION_PERCENT = 4.0f;

static const unsigned long CONTROL_INTERVAL_MS = 50;
static const unsigned long PRINT_INTERVAL_MS = 250;
static const unsigned long WHEEL_STALL_TIMEOUT_MS = 2500;
static const unsigned long COMPLETE_TEST_TIMEOUT_MS = 30000;
static const unsigned long BRAKE_TIME_MS = 100;

static const uint8_t MOTOR_IDS[4]   = { MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR };
static const uint8_t ENCODER_IDS[4] = { ENCODER_FL, ENCODER_FR, ENCODER_RL, ENCODER_RR };
static const char *WHEEL_NAMES[4]   = { "Front Left", "Front Right", "Rear Left", "Rear Right" };

float clampFloat(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

float millimetresPerTick(uint8_t wheel)
{
    return WHEEL_CIRCUMFERENCE_MM /
           (SLOT_COUNTS[wheel] * COUNTED_EDGES_PER_SLOT);
}

float wheelDistanceMM(uint8_t wheel, long ticks)
{
    return ticks * millimetresPerTick(wheel);
}

void readAllTicks(long ticks[4])
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        ticks[i] = Encoders.GetCount(ENCODER_IDS[i]);
    }
}

// Effective track width for the requested turn direction.
float trackWidthFor(bool turnLeft)
{
    return turnLeft ? TRACK_WIDTH_LEFT_MM : TRACK_WIDTH_RIGHT_MM;
}

// Arc length each wheel must travel for a given turn angle (degrees).
//   d_wheel = pi * W * theta / 360
float wheelArcForAngle(float angleDeg, bool turnLeft)
{
    return (PI * trackWidthFor(turnLeft) * fabsf(angleDeg)) / 360.0f;
}

void stopAndDisable()
{
    Motor.StopAll();
    Motor.Enable(false);
}

void printTurnHeader(float angleDeg, bool turnLeft, float wheelArcMM)
{
    Serial.println();
    Serial.println("==============================================");
    Serial.println(" In-place turn test");
    Serial.println("==============================================");
    Serial.print("Battery: ");
    Serial.print(Motor.GetBatteryVoltage(), 2);
    Serial.println(" V");
    Serial.print("Requested turn: ");
    Serial.print(fabsf(angleDeg), 1);
    Serial.print(" deg ");
    Serial.println(turnLeft ? "LEFT" : "RIGHT");
    Serial.print("Track width (calibration, ");
    Serial.print(turnLeft ? "LEFT" : "RIGHT");
    Serial.print("): ");
    Serial.print(trackWidthFor(turnLeft), 1);
    Serial.println(" mm");
    Serial.print("Each wheel arc target: ");
    Serial.print(wheelArcMM, 2);
    Serial.println(" mm");
    for (uint8_t i = 0; i < 4; ++i)
    {
        Serial.print("  ");
        Serial.print(WHEEL_NAMES[i]);
        Serial.print(": ");
        Serial.print(millimetresPerTick(i), 3);
        Serial.print(" mm/tick, approx ");
        Serial.print(lroundf(wheelArcMM / millimetresPerTick(i)));
        Serial.println(" target ticks");
    }
    Serial.println("==============================================");
}

// direction[] gives the commanded spin per wheel for the requested turn.
// For a LEFT turn: left wheels backward, right wheels forward.
// For a RIGHT turn: left wheels forward, right wheels backward.
bool turnInPlace(float angleDeg, bool turnLeft)
{
    if (fabsf(angleDeg) < 1.0f || !isfinite(angleDeg))
    {
        Serial.println("ERROR: Invalid or too small an angle.");
        return false;
    }

    const float wheelArcMM = wheelArcForAngle(angleDeg, turnLeft);
    printTurnHeader(angleDeg, turnLeft, wheelArcMM);

    Motor.StopAll();
    for (uint8_t i = 0; i < 4; ++i)
    {
        Encoders.Reset(ENCODER_IDS[i]);
    }

    // Per-wheel commanded direction. Index order: FL, FR, RL, RR.
    // onLeft = wheel 0 (FL) or 2 (RL).
    int wheelDir[4];
    for (uint8_t i = 0; i < 4; ++i)
    {
        const bool onLeft = (i == 0 || i == 2);
        if (turnLeft)
            wheelDir[i] = onLeft ? Backward : Forward;
        else
            wheelDir[i] = onLeft ? Forward : Backward;
    }

    long  ticks[4]         = {0, 0, 0, 0};
    long  previousTicks[4] = {0, 0, 0, 0};
    float distanceMM[4]    = {0, 0, 0, 0};   // magnitude of arc travelled
    float commands[4]      = {0, 0, 0, 0};

    Motor.Enable(true);
    const unsigned long startMS = millis();
    unsigned long lastControlMS = startMS;
    unsigned long lastPrintMS = startMS;
    unsigned long lastTickMS[4] = {startMS, startMS, startMS, startMS};

    Serial.println();
    Serial.print("Turning ");
    Serial.println(turnLeft ? "LEFT..." : "RIGHT...");

    while (true)
    {
        const unsigned long now = millis();
        const unsigned long elapsedMS = now - startMS;

        if (elapsedMS >= COMPLETE_TEST_TIMEOUT_MS)
        {
            stopAndDisable();
            Serial.println("ERROR: Overall timeout. All motors stopped.");
            return false;
        }

        if (now - lastControlMS < CONTROL_INTERVAL_MS)
        {
            delay(1);
            continue;
        }
        lastControlMS = now;

        readAllTicks(ticks);
        for (uint8_t i = 0; i < 4; ++i)
        {
            distanceMM[i] = wheelDistanceMM(i, ticks[i]);
            if (ticks[i] != previousTicks[i])
            {
                previousTicks[i] = ticks[i];
                lastTickMS[i] = now;
            }
        }

        // Average arc travelled across all four wheels.
        const float averageArcMM =
            (distanceMM[0] + distanceMM[1] + distanceMM[2] + distanceMM[3]) / 4.0f;

        if (averageArcMM >= wheelArcMM)
        {
            break;  // reached the arc target -> brake together
        }

        // stall watchdog
        for (uint8_t i = 0; i < 4; ++i)
        {
            if (now - lastTickMS[i] > WHEEL_STALL_TIMEOUT_MS)
            {
                stopAndDisable();
                Serial.print("ERROR: No ticks from ");
                Serial.print(WHEEL_NAMES[i]);
                Serial.println(". All motors stopped.");
                return false;
            }
        }

        // ---- side balance: keep both sides turning at equal arc rate ----
        const float leftMM  = (distanceMM[0] + distanceMM[2]) / 2.0f;
        const float rightMM = (distanceMM[1] + distanceMM[3]) / 2.0f;
        const float balanceError = leftMM - rightMM;  // + => left ahead
        const float balanceCorrection = clampFloat(
            BALANCE_KP * balanceError,
            -MAX_BALANCE_CORRECTION_PERCENT,
            MAX_BALANCE_CORRECTION_PERCENT);

        // ---- approach slowdown on the remaining arc ----
        const float remainingMM = wheelArcMM - averageArcMM;
        float baseSpeed = CRUISE_SPEED_PERCENT;
        if (remainingMM < SLOWDOWN_ARC_MM)
        {
            const float progress = clampFloat(remainingMM / SLOWDOWN_ARC_MM, 0.0f, 1.0f);
            baseSpeed = APPROACH_SPEED_PERCENT +
                        (CRUISE_SPEED_PERCENT - APPROACH_SPEED_PERCENT) * progress;
        }

        for (uint8_t i = 0; i < 4; ++i)
        {
            const bool onLeft = (i == 0 || i == 2);
            const float sideMM = onLeft ? leftMM : rightMM;

            // front/rear evenness within this side
            const float withinSideCorrection = clampFloat(
                SAME_SIDE_KP * (sideMM - distanceMM[i]),
                -MAX_SAME_SIDE_CORRECTION_PERCENT,
                MAX_SAME_SIDE_CORRECTION_PERCENT);

            // if the left side is ahead (balanceError > 0), slow left / speed right
            const float balanceTrim = onLeft ? -balanceCorrection : balanceCorrection;

            commands[i] = clampFloat(baseSpeed + balanceTrim + withinSideCorrection,
                                     MIN_ACTIVE_SPEED_PERCENT,
                                     MAX_SPEED_PERCENT);

            if (!Motor.Move(MOTOR_IDS[i], wheelDir[i], commands[i]))
            {
                stopAndDisable();
                Serial.println("ERROR: Motor command rejected; check battery.");
                return false;
            }
        }

        if (now - lastPrintMS >= PRINT_INTERVAL_MS)
        {
            lastPrintMS = now;
            Serial.println();
            Serial.print("Time ");
            Serial.print(elapsedMS);
            Serial.print(" ms | mean arc ");
            Serial.print(averageArcMM, 1);
            Serial.print(" / ");
            Serial.print(wheelArcMM, 1);
            Serial.print(" mm | balance L-R ");
            Serial.print(balanceError, 1);
            Serial.println(" mm");
            for (uint8_t i = 0; i < 4; ++i)
            {
                Serial.print("  ");
                Serial.print(WHEEL_NAMES[i]);
                Serial.print(": ");
                Serial.print(ticks[i]);
                Serial.print(" ticks, ");
                Serial.print(distanceMM[i], 1);
                Serial.print(" mm | cmd ");
                Serial.print(commands[i], 1);
                Serial.println("%");
            }
        }
    }

    // brake together, then release
    Motor.Motor(Brake, 255);
    delay(BRAKE_TIME_MS);
    Motor.StopAll();
    delay(150);
    readAllTicks(ticks);
    stopAndDisable();

    for (uint8_t i = 0; i < 4; ++i)
    {
        distanceMM[i] = wheelDistanceMM(i, ticks[i]);
    }
    const float meanArc =
        (distanceMM[0] + distanceMM[1] + distanceMM[2] + distanceMM[3]) / 4.0f;
    // Back-calculate the angle the encoders think we turned (same width used).
    const float estimatedAngle =
        (meanArc * 360.0f) / (PI * trackWidthFor(turnLeft));

    Serial.println();
    Serial.println("==============================================");
    Serial.println(" Turn complete");
    Serial.println("==============================================");
    for (uint8_t i = 0; i < 4; ++i)
    {
        Serial.print(WHEEL_NAMES[i]);
        Serial.print(": ");
        Serial.print(ticks[i]);
        Serial.print(" ticks (");
        Serial.print(distanceMM[i], 1);
        Serial.println(" mm arc)");
    }
    Serial.print("Mean wheel arc: ");
    Serial.print(meanArc, 1);
    Serial.println(" mm");
    Serial.print("Encoder-estimated angle: ");
    Serial.print(estimatedAngle, 1);
    Serial.println(" deg");
    Serial.print("Requested angle: ");
    Serial.print(fabsf(angleDeg), 1);
    Serial.println(" deg");
    Serial.println("Measure the ACTUAL turn on the floor and adjust");
    Serial.println("TRACK_WIDTH_MM: W_new = W_old * requested / actual.");
    return true;
}

bool beginTurnDrive()
{
    if (initialized) return true;

    Motor.Init();
    Motor.StopAll();
    Encoders.Init();

    if (!Motor.RefreshBatteryVoltage(32))
    {
        Serial.println("ERROR: Invalid battery reading; test cancelled.");
        stopAndDisable();
        return false;
    }
    initialized = true;
    return true;
}

} // namespace

// ---- Public API ----
bool turnLeft(float angleDeg)
{
    if (!beginTurnDrive()) return false;
    return turnInPlace(angleDeg, true);
}

bool turnRight(float angleDeg)
{
    if (!beginTurnDrive()) return false;
    return turnInPlace(angleDeg, false);
}
#include "StraightDrive.h"
#include <Arduino.h>
#include <Motor.h>
#include <encoder.h>
#include <math.h>

namespace {
motor Motor;
encoder Encoders;
bool initialized = false;

// Distance measured on the floor from the starting wheel axle.
static const float WHEEL_DIAMETER_MM = 60.0f;

// Wheel order: front left, front right, rear left, rear right.
// Only the front-right wheel currently has a 20-slot encoder disc.
static const uint8_t SLOT_COUNTS[4] = {16, 20, 16, 16};

// One RISING edge counted per slot. Set to 2 if the encoder library counts
// both RISING and FALLING edges; use the same setting for all four sensors.
static const uint8_t COUNTED_EDGES_PER_SLOT = 2;

static const float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;

// Motor.Move() uses a 0-100% command and maps it to battery-aware PWM.
static const float CRUISE_SPEED_PERCENT = 5.0f;
static const float APPROACH_SPEED_PERCENT = 2.0f;
static const float MIN_ACTIVE_SPEED_PERCENT = 1.0f;
static const float MAX_SPEED_PERCENT = 30.0f;
static const float SLOWDOWN_DISTANCE_MM = 140.0f;

// Side averages are compared in millimetres, not ticks.
// Positive left-right error means the rover tends to turn right.
static const float HEADING_KP = 0.13f;  // command % per mm of side error
static const float HEADING_KI = 0.035f; // command % per (mm * second)
static const float MAX_HEADING_CORRECTION_PERCENT = 8.0f;
static const float MAX_HEADING_INTEGRAL_MM_S = 220.0f;
static const float SAME_SIDE_KP = 0.085f; // command % per mm of wheel error
static const float MAX_SAME_SIDE_CORRECTION_PERCENT = 4.0f;

// Ground-test trim: positive values steer left (less left, more right).
// Start with 1%, then tune in small steps using the actual path on the floor.
static const float LEFT_STATIC_TRIM_PERCENT = 10.0f;

// The side-average heading PI loop would learn to cancel a permanent trim.
// While calibrating against the observed path, keep it off; encoder-based
// distance stopping and front/rear correction on each side remain active.
static const bool TUNE_LEFT_STATIC_TRIM = true;

static const unsigned long CONTROL_INTERVAL_MS = 50;
static const unsigned long PRINT_INTERVAL_MS = 250;
static const unsigned long WHEEL_STALL_TIMEOUT_MS = 2500;
static const unsigned long COMPLETE_TEST_TIMEOUT_MS = 30000;
static const unsigned long BRAKE_TIME_MS = 100;

static const uint8_t MOTOR_IDS[4] =
{
    MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR
};

static const uint8_t ENCODER_IDS[4] =
{
    ENCODER_FL, ENCODER_FR, ENCODER_RL, ENCODER_RR
};

static const char *WHEEL_NAMES[4] =
{
    "Front Left", "Front Right", "Rear Left", "Rear Right"
};

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

long targetTicksForWheel(uint8_t wheel, float distanceMM)
{
    return static_cast<long>(lroundf(distanceMM / millimetresPerTick(wheel)));
}

void readAllTicks(long ticks[4])
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        ticks[i] = Encoders.GetCount(ENCODER_IDS[i]);
    }
}

float meanDistanceMM(const float distanceMM[4])
{
    return (distanceMM[0] + distanceMM[1] +
            distanceMM[2] + distanceMM[3]) / 4.0f;
}

void stopAndDisable()
{
    Motor.StopAll();
    Motor.Enable(false);
}

void printHeader(float targetDistanceMM)
{
    Serial.println();
    Serial.println("==============================================");
    Serial.println(" Straight-line distance test: mixed encoders");
    Serial.println("==============================================");
    Serial.print("Battery: ");
    Serial.print(Motor.GetBatteryVoltage(), 2);
    Serial.println(" V");
    Serial.print("Dynamic PWM range: ");
    Serial.print(Motor.GetMinimumPWM());
    Serial.print(" to ");
    Serial.println(Motor.GetMaximumPWM());
    Serial.print("Leftward static trim: ");
    Serial.print(LEFT_STATIC_TRIM_PERCENT, 1);
    Serial.println("% (left -= trim, right += trim)");
    Serial.print("Side-to-side heading PI: ");
    Serial.println(TUNE_LEFT_STATIC_TRIM ? "OFF (trim tuning)" : "ON");
    Serial.print("Wheel diameter: ");
    Serial.print(WHEEL_DIAMETER_MM, 1);
    Serial.println(" mm");
    Serial.print("Wheel circumference: ");
    Serial.print(WHEEL_CIRCUMFERENCE_MM, 2);
    Serial.println(" mm");
    Serial.print("Requested travel: ");
    Serial.print(targetDistanceMM, 1);
    Serial.println(" mm");
    Serial.print("Counted edges per slot: ");
    Serial.println(COUNTED_EDGES_PER_SLOT);

    for (uint8_t i = 0; i < 4; ++i)
    {
        Serial.print("  ");
        Serial.print(WHEEL_NAMES[i]);
        Serial.print(": ");
        Serial.print(SLOT_COUNTS[i]);
        Serial.print(" slots, ");
        Serial.print(millimetresPerTick(i), 3);
        Serial.print(" mm/tick, approximately ");
        Serial.print(targetTicksForWheel(i, targetDistanceMM));
        Serial.println(" target ticks");
    }
    Serial.println("==============================================");
}

void printProgress(unsigned long elapsedMS,
                   const long ticks[4],
                   const float distanceMM[4],
                   const float commands[4],
                   float headingErrorMM,
                   float headingCorrectionPercent,
                   float targetDistanceMM)
{
    Serial.println();
    Serial.print("Time ");
    Serial.print(elapsedMS);
    Serial.print(" ms | mean travel ");
    Serial.print(meanDistanceMM(distanceMM), 1);
    Serial.print(" / ");
    Serial.print(targetDistanceMM, 1);
    Serial.println(" mm");
    Serial.print("  Left - right distance: ");
    Serial.print(headingErrorMM, 1);
    Serial.print(" mm | heading feedback: ");
    Serial.print(headingCorrectionPercent, 1);
    Serial.print("% | leftward static trim: ");
    Serial.print(LEFT_STATIC_TRIM_PERCENT, 1);
    Serial.println("%");

    for (uint8_t i = 0; i < 4; ++i)
    {
        Serial.print("  ");
        Serial.print(WHEEL_NAMES[i]);
        Serial.print(": ");
        Serial.print(ticks[i]);
        Serial.print(" ticks, ");
        Serial.print(distanceMM[i], 1);
        Serial.print(" mm | command ");
        Serial.print(commands[i], 1);
        Serial.println("%");
    }
}

bool moveForwardDistance(float targetDistanceMM)
{
    if (targetDistanceMM < millimetresPerTick(0) ||
        !isfinite(targetDistanceMM))
    {
        Serial.println("ERROR: Invalid or too short a distance.");
        return false;
    }

    Motor.StopAll();
    for (uint8_t i = 0; i < 4; ++i)
    {
        Encoders.Reset(ENCODER_IDS[i]);
    }

    long ticks[4] = {0, 0, 0, 0};
    long previousTicks[4] = {0, 0, 0, 0};
    float distanceMM[4] = {0, 0, 0, 0};
    float commands[4] = {0, 0, 0, 0};

    Motor.Enable(true);
    const unsigned long startMS = millis();
    unsigned long lastControlMS = startMS;
    unsigned long lastPrintMS = startMS;
    unsigned long lastTickMS[4] = {startMS, startMS, startMS, startMS};
    float headingIntegralMMs = 0.0f;

    Serial.println();
    Serial.println("Moving forward...");

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

        const unsigned long controlElapsedMS = now - lastControlMS;
        if (controlElapsedMS < CONTROL_INTERVAL_MS)
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

        const float averageMM = meanDistanceMM(distanceMM);
        if (averageMM >= targetDistanceMM)
        {
            break; // Brake all wheels together to avoid a final steering turn.
        }

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

        const float leftMM = (distanceMM[0] + distanceMM[2]) / 2.0f;
        const float rightMM = (distanceMM[1] + distanceMM[3]) / 2.0f;
        const float headingErrorMM = leftMM - rightMM;

        float headingCorrection = 0.0f;
        if (!TUNE_LEFT_STATIC_TRIM)
        {
            // Integrate measured distance error using the actual loop interval.
            // Limit its effect if the wheel stalls or the correction saturates.
            headingIntegralMMs += headingErrorMM * (controlElapsedMS / 1000.0f);
            headingIntegralMMs = clampFloat(headingIntegralMMs,
                                            -MAX_HEADING_INTEGRAL_MM_S,
                                            MAX_HEADING_INTEGRAL_MM_S);
            headingCorrection = clampFloat(
                HEADING_KP * headingErrorMM + HEADING_KI * headingIntegralMMs,
                -MAX_HEADING_CORRECTION_PERCENT,
                MAX_HEADING_CORRECTION_PERCENT);
        }

        const float remainingMM = targetDistanceMM - averageMM;
        float baseSpeed = CRUISE_SPEED_PERCENT;
        if (remainingMM < SLOWDOWN_DISTANCE_MM)
        {
            const float progress = clampFloat(remainingMM / SLOWDOWN_DISTANCE_MM,
                                              0.0f, 1.0f);
            baseSpeed = APPROACH_SPEED_PERCENT +
                        (CRUISE_SPEED_PERCENT - APPROACH_SPEED_PERCENT) *
                        progress;
        }

        for (uint8_t i = 0; i < 4; ++i)
        {
            const bool onLeft = (i == 0 || i == 2);
            const float sideMM = onLeft ? leftMM : rightMM;
            const float withinSideCorrection = clampFloat(
                SAME_SIDE_KP * (sideMM - distanceMM[i]),
                -MAX_SAME_SIDE_CORRECTION_PERCENT,
                MAX_SAME_SIDE_CORRECTION_PERCENT);
            const float headingTrim = onLeft
                                          ? -headingCorrection
                                          : headingCorrection;

            const float staticTrim = onLeft
                                         ? -LEFT_STATIC_TRIM_PERCENT
                                         : LEFT_STATIC_TRIM_PERCENT;

            commands[i] = clampFloat(baseSpeed + headingTrim +
                                         staticTrim + withinSideCorrection,
                                     MIN_ACTIVE_SPEED_PERCENT,
                                     MAX_SPEED_PERCENT);
            if (!Motor.Move(MOTOR_IDS[i], Forward, commands[i]))
            {
                stopAndDisable();
                Serial.println("ERROR: Motor command rejected; check battery.");
                return false;
            }
        }

        if (now - lastPrintMS >= PRINT_INTERVAL_MS)
        {
            lastPrintMS = now;
            printProgress(elapsedMS, ticks, distanceMM,
                          commands, headingErrorMM, headingCorrection,
                          targetDistanceMM);
        }
    }

    // Brief active brake reduces coast after the final tick, then releases.
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

    Serial.println();
    Serial.println("==============================================");
    Serial.println(" Motor stop complete");
    Serial.println("==============================================");
    for (uint8_t i = 0; i < 4; ++i)
    {
        Serial.print(WHEEL_NAMES[i]);
        Serial.print(": ");
        Serial.print(ticks[i]);
        Serial.print(" ticks (");
        Serial.print(distanceMM[i], 1);
        Serial.println(" mm encoder estimate)");
    }
    Serial.print("Average encoder travel: ");
    Serial.print(meanDistanceMM(distanceMM), 1);
    Serial.println(" mm");
    Serial.print("Requested travel: ");
    Serial.print(targetDistanceMM, 1);
    Serial.println(" mm");
    Serial.println("Measure the actual axle displacement on the floor.");
    return true;
}

} // namespace

bool beginStraightDrive()
{
    if (initialized)
    {
        return true;
    }

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

bool goStraight(float distanceMM)
{
    if (!beginStraightDrive())
    {
        return false;
    }

    printHeader(distanceMM);
    return moveForwardDistance(distanceMM);
}

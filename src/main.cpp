#include <Arduino.h>
#include <Motor.h>
#include <encoder.h>

motor Motor;
encoder Encoders;

// Move() uses percentages, not raw PWM values:
//   0%   = stop
//   1%   = dynamically calculated minimum runnable PWM
//   100% = dynamically calculated maximum safe PWM
static const int MIN_SPEED_PERCENT = 1;
static const int MAX_SPEED_PERCENT = 61;
static const int SPEED_STEP_PERCENT = 30;
static const unsigned long STEP_TIME_MS = 2000;

void printStepResult(int speedPercent, long pulses)
{
    float pulsesPerSecond = pulses * 1000.0f / STEP_TIME_MS;

    Serial.print("  Speed ");
    Serial.print(speedPercent);
    Serial.print("% -> ");
    Serial.print(pulses);
    Serial.print(" pulses, ");
    Serial.print(pulsesPerSecond, 1);
    Serial.println(" pulses/s");
}

bool runSpeedStep(uint8_t motorID, uint8_t encoderID, int speedPercent)
{
    Encoders.Reset(encoderID);

    if (!Motor.Move(motorID, Forward, speedPercent))
    {
        Serial.println("  ERROR: Motor command rejected due to an invalid battery reading.");
        return false;
    }

    delay(STEP_TIME_MS);

    long pulses = Encoders.GetCount(encoderID);
    printStepResult(speedPercent, pulses);
    return true;
}

bool testMotor(uint8_t motorID, uint8_t encoderID, const char *name)
{
    Serial.println();
    Serial.print("Testing ");
    Serial.println(name);

    // Take a fresh, averaged battery measurement before each motor test.
    if (!Motor.RefreshBatteryVoltage(16))
    {
        Serial.print("  Invalid battery reading: ");
        Serial.print(Motor.GetBatteryVoltage(), 2);
        Serial.println(" V. Test aborted.");
        Motor.StopAll();
        Motor.Enable(false);
        return false;
    }

    Serial.print("  Battery voltage: ");
    Serial.print(Motor.GetBatteryVoltage(), 2);
    Serial.println(" V");

    Serial.print("  Dynamic PWM range: ");
    Serial.print(Motor.GetMinimumPWM());
    Serial.print(" to ");
    Serial.println(Motor.GetMaximumPWM());

    Motor.Enable(true);

    Serial.println("  Ramping up:");

    // Test the exact minimum runnable command first.
    if (!runSpeedStep(motorID, encoderID, MIN_SPEED_PERCENT))
    {
        Motor.StopAll();
        Motor.Enable(false);
        return false;
    }

    for (int speed = SPEED_STEP_PERCENT;
         speed <= MAX_SPEED_PERCENT;
         speed += SPEED_STEP_PERCENT)
    {
        if (!runSpeedStep(motorID, encoderID, speed))
        {
            Motor.StopAll();
            Motor.Enable(false);
            return false;
        }
    }

    Serial.println("  Ramping down:");

    // Do not repeat 100%; descend through 90%, 80%, ... 10%.
    for (int speed = MAX_SPEED_PERCENT - SPEED_STEP_PERCENT;
         speed >= SPEED_STEP_PERCENT;
         speed -= SPEED_STEP_PERCENT)
    {
        if (!runSpeedStep(motorID, encoderID, speed))
        {
            Motor.StopAll();
            Motor.Enable(false);
            return false;
        }
    }

    // Finish at the exact minimum runnable command.
    if (!runSpeedStep(motorID, encoderID, MIN_SPEED_PERCENT))
    {
        Motor.StopAll();
        Motor.Enable(false);
        return false;
    }

    Motor.Move(motorID, Stop, 0);
    Motor.Enable(false);

    Serial.print(name);
    Serial.println(" complete.");
    delay(1000);
    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" One-wheel-at-a-time Move() ramp test");
    Serial.println("========================================");
    Serial.println("Lift the rover so all wheels are off the ground.");
    Serial.println("Each motor will ramp from 1% to 100% and back.");
    Serial.println("Move() maps this range to safe PWM limits using battery voltage.");

    Motor.Init();
    Motor.StopAll();
    Encoders.Init();

    Serial.println("Starting in 3 seconds...");
    delay(3000);

    if (!testMotor(MOTOR_FL, ENCODER_FL, "Front Left")) return;
    if (!testMotor(MOTOR_FR, ENCODER_FR, "Front Right")) return;
    if (!testMotor(MOTOR_RL, ENCODER_RL, "Rear Left")) return;
    if (!testMotor(MOTOR_RR, ENCODER_RR, "Rear Right")) return;

    Motor.StopAll();
    Motor.Enable(false);

    Serial.println();
    Serial.println("All motor ramp tests complete.");
    Serial.println("Reset the board to run the test again.");
}

void loop()
{
    // The test runs once in setup().
}

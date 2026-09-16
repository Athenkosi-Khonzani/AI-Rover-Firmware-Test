#include <Arduino.h>
#include <Motor.h>
#include <encoder.h>

motor Motor;
encoder Encoders;

static const int MIN_PWM = 20;
static const int MAX_PWM = 140;
static const int PWM_STEP = 10;
static const unsigned long STEP_TIME_MS = 500;

void printStepResult(int pwm, long pulses)
{
    float pulsesPerSecond = pulses * 1000.0f / STEP_TIME_MS;

    Serial.print("  PWM ");
    Serial.print(pwm);
    Serial.print(" -> ");
    Serial.print(pulses);
    Serial.print(" pulses, ");
    Serial.print(pulsesPerSecond, 1);
    Serial.println(" pulses/s");
}

void runSpeedStep(uint8_t motorID, uint8_t encoderID, int pwm)
{
    Encoders.Reset(encoderID);
    Motor.Motor(motorID, Forward, pwm);
    delay(STEP_TIME_MS);

    long pulses = Encoders.GetCount(encoderID);
    printStepResult(pwm, pulses);
}

void testMotor(uint8_t motorID, uint8_t encoderID, const char *name)
{
    Serial.println();
    Serial.print("Testing ");
    Serial.println(name);

    Motor.Enable(true);

    // Ramp from PWM 20 up to PWM 140.
    Serial.println("  Ramping up:");
    for (int pwm = MIN_PWM; pwm <= MAX_PWM; pwm += PWM_STEP)
    {
        runSpeedStep(motorID, encoderID, pwm);
    }

    // Ramp from PWM 140 back down to PWM 20.
    Serial.println("  Ramping down:");
    for (int pwm = MAX_PWM - PWM_STEP; pwm >= MIN_PWM; pwm -= PWM_STEP)
    {
        runSpeedStep(motorID, encoderID, pwm);
    }

    Motor.Motor(motorID, Stop, 0);
    Motor.Enable(false);

    Serial.print(name);
    Serial.println(" complete.");
    delay(1000);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" One-wheel-at-a-time motor ramp test");
    Serial.println("========================================");
    Serial.println("Lift the rover so all wheels are off the ground.");
    Serial.println("Each motor will ramp from PWM 20 to 140 and back.");

    Motor.Init();
    Motor.StopAll();
    Encoders.Init();

    Serial.println("Starting in 3 seconds...");
    delay(3000);

    testMotor(MOTOR_FL, ENCODER_FL, "Front Left");
    testMotor(MOTOR_FR, ENCODER_FR, "Front Right");
    testMotor(MOTOR_RL, ENCODER_RL, "Rear Left");
    testMotor(MOTOR_RR, ENCODER_RR, "Rear Right");

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

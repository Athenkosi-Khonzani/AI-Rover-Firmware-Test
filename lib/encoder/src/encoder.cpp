#include "encoder.h"
#include "Arduino.h"

volatile uint32_t encoderFLCount = 0;
volatile uint32_t encoderFRCount = 0;
volatile uint32_t encoderRLCount = 0;
volatile uint32_t encoderRRCount = 0;

// Protect pulse-counter updates/read-reset operations between the ISR and main
// code on the dual-core ESP32.
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoderFLHandler()
{
    portENTER_CRITICAL_ISR(&encoderMux);
    encoderFLCount++;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR encoderFRHandler()
{
    portENTER_CRITICAL_ISR(&encoderMux);
    encoderFRCount++;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR encoderRLHandler()
{
    portENTER_CRITICAL_ISR(&encoderMux);
    encoderRLCount++;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR encoderRRHandler()
{
    portENTER_CRITICAL_ISR(&encoderMux);
    encoderRRCount++;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void encoder::Init()
{
    // GPIO34, 35, 36 and 39 have no internal pull-up/down resistors.
    pinMode(ENCODER_FL_PIN, INPUT);
    pinMode(ENCODER_FR_PIN, INPUT);
    pinMode(ENCODER_RL_PIN, INPUT);
    pinMode(ENCODER_RR_PIN, INPUT);

    ResetAll();

    // One RISING edge is counted per encoder pulse.
    attachInterrupt(digitalPinToInterrupt(ENCODER_FL_PIN), encoderFLHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_FR_PIN), encoderFRHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RL_PIN), encoderRLHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RR_PIN), encoderRRHandler, RISING);
}

uint32_t encoder::GetCount(uint8_t encoderID)
{
    uint32_t value = 0;

    portENTER_CRITICAL(&encoderMux);
    switch (encoderID)
    {
        case ENCODER_FL:
            value = encoderFLCount;
            break;
        case ENCODER_FR:
            value = encoderFRCount;
            break;
        case ENCODER_RL:
            value = encoderRLCount;
            break;
        case ENCODER_RR:
            value = encoderRRCount;
            break;
        default:
            value = 0;
            break;
    }
    portEXIT_CRITICAL(&encoderMux);

    return value;
}

void encoder::Reset(uint8_t encoderID)
{
    portENTER_CRITICAL(&encoderMux);
    switch (encoderID)
    {
        case ENCODER_FL:
            encoderFLCount = 0;
            break;
        case ENCODER_FR:
            encoderFRCount = 0;
            break;
        case ENCODER_RL:
            encoderRLCount = 0;
            break;
        case ENCODER_RR:
            encoderRRCount = 0;
            break;
        default:
            break;
    }
    portEXIT_CRITICAL(&encoderMux);
}

void encoder::ResetAll()
{
    portENTER_CRITICAL(&encoderMux);
    encoderFLCount = 0;
    encoderFRCount = 0;
    encoderRLCount = 0;
    encoderRRCount = 0;
    portEXIT_CRITICAL(&encoderMux);
}

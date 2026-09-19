#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Single-channel LM393 wheel encoder inputs from the custom PCB.
// GPIO34-39 are input-only pins and have no internal pull-ups, so pinMode is
// deliberately INPUT. The PCB provides optional external 10k pull-ups.
// -----------------------------------------------------------------------------
#define ENCODER_FL_PIN 34
#define ENCODER_FR_PIN 39
#define ENCODER_RL_PIN 35
#define ENCODER_RR_PIN 36

#define ENCODER_FL 1
#define ENCODER_FR 2
#define ENCODER_RL 3
#define ENCODER_RR 4

// Raw pulse counters, incremented by the interrupt handlers below.
extern volatile uint32_t encoderFLCount;
extern volatile uint32_t encoderFRCount;
extern volatile uint32_t encoderRLCount;
extern volatile uint32_t encoderRRCount;

// Interrupt handlers. These are intentionally exposed so they are easy to find
// and can later be extended for odometry/control work.
void IRAM_ATTR encoderFLHandler();
void IRAM_ATTR encoderFRHandler();
void IRAM_ATTR encoderRLHandler();
void IRAM_ATTR encoderRRHandler();

class encoder
{
    public:
        void Init();

        uint32_t GetCount(uint8_t encoderID);
        void Reset(uint8_t encoderID);
        void ResetAll();
};

#endif

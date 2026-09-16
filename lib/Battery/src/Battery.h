#ifndef _BATTERY_H_
#define _BATTERY_H_

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Battery monitor for the custom ACEBOTT PCB.
//
// Hardware:
//   +BATT -> 47k -> GPIO32 -> 15k -> GND
//                         |
//                       100nF
//                         |
//                        GND
//
// GPIO32 is ADC1_CH4 on the ESP32.
//
// IMPORTANT ABOUT CURRENT:
// There is no current-sense resistor or current-sense IC on this PCB.
// Current is therefore ESTIMATED from battery voltage sag:
//
//   I_load ~= I_baseline + (V_baseline - V_load) / R_effective
//
// R_effective must be measured/estimated for the actual battery + leads +
// connectors + ACEBOTT VIN path. This is a rough load estimate, not a true
// current measurement and it will not accurately capture short current spikes.
// -----------------------------------------------------------------------------

#define BATTERY_SENSE_PIN 32

class battery
{
    public:
        void Init();

        // Raw ESP32 ADC reading (0-4095 at 12-bit resolution), averaged.
        uint16_t ReadRawADC(uint8_t samples = 16);

        // Voltage actually present on GPIO32 AFTER the resistor divider.
        float ReadADCVoltage(uint8_t samples = 16);

        // Reconstructed battery/+BATT terminal voltage BEFORE the divider.
        // This is the main voltage function normally used by the application.
        float ReadVoltage(uint8_t samples = 16);

        // Effective series resistance used for voltage-sag current estimation.
        // Use the TOTAL effective resistance seen at the PCB measurement point:
        // battery internal resistance + leads/connectors + VIN/header path.
        void SetEffectiveResistance(float resistanceOhms);
        float GetEffectiveResistance();

        // Save the light-load/resting battery voltage immediately before a
        // known load step, for example just before starting the motors.
        float CaptureBaseline(uint8_t samples = 32);
        float GetBaselineVoltage();
        bool HasBaseline();
        void ClearBaseline();

        // Estimate current from the saved baseline and a new ADC reading.
        // If baselineCurrentA is left at zero, the result is the ADDITIONAL
        // current relative to the baseline condition.
        float EstimateCurrent(uint8_t samples = 32,
                              float baselineCurrentA = 0.0f);

        // Same calculation when the loaded voltage has already been measured.
        float EstimateCurrentFromVoltage(float loadedVoltage,
                                         float baselineCurrentA = 0.0f);

        // Stateless version useful for experiments/log processing.
        float EstimateCurrentFromVoltages(float baselineVoltage,
                                          float loadedVoltage,
                                          float baselineCurrentA = 0.0f);

        // Optional rough LiPo state-of-charge estimate from RESTING voltage.
        // Uses a generic per-cell LiPo OCV table. Replace/calibrate the table in
        // Battery.cpp once the exact battery discharge curve is known.
        void SetCellCount(uint8_t cells);
        uint8_t GetCellCount();
        float EstimateSOC(float packVoltage);
        float EstimateSOCFromBaseline();

    private:
        float _effectiveResistanceOhms = 0.0f;
        float _baselineVoltage = 0.0f;
        bool _baselineValid = false;
        uint8_t _cellCount = 2;
};

#endif

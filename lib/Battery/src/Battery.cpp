#include "Battery.h"
#include "Arduino.h"
#include <math.h>

// PCB divider: +BATT -> 47k -> ADC -> 15k -> GND.
static const float R_DIVIDER_TOP_KOHM = 47.0f;
static const float R_DIVIDER_BOTTOM_KOHM = 15.0f;
static const float BATTERY_DIVIDER_GAIN =
    (R_DIVIDER_TOP_KOHM + R_DIVIDER_BOTTOM_KOHM) / R_DIVIDER_BOTTOM_KOHM;

// Generic LiPo open-circuit-voltage approximation PER CELL.
// This is intentionally only a starting point. The real voltage/SOC curve
// depends on cell chemistry, temperature, age, load history and the exact pack.
struct LiPoSOCPoint
{
    float voltsPerCell;
    float percent;
};

static const LiPoSOCPoint LIPO_SOC_TABLE[] =
{
    {4.20f, 100.0f},
    {4.15f,  95.0f},
    {4.10f,  90.0f},
    {4.05f,  85.0f},
    {4.00f,  80.0f},
    {3.95f,  70.0f},
    {3.90f,  60.0f},
    {3.85f,  50.0f},
    {3.80f,  40.0f},
    {3.75f,  30.0f},
    {3.70f,  20.0f},
    {3.60f,  10.0f},
    {3.50f,   0.0f}
};

static const size_t LIPO_SOC_TABLE_SIZE =
    sizeof(LIPO_SOC_TABLE) / sizeof(LIPO_SOC_TABLE[0]);

void battery::Init()
{
    pinMode(BATTERY_SENSE_PIN, INPUT);

    // GPIO32 is ADC1, so it remains usable even when ESP32 Wi-Fi is active.
    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_SENSE_PIN, ADC_11db);

    ClearBaseline();
}

uint16_t battery::ReadRawADC(uint8_t samples)
{
    if (samples == 0)
    {
        samples = 1;
    }

    uint32_t total = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        total += analogRead(BATTERY_SENSE_PIN);
        delay(1);
    }

    return static_cast<uint16_t>(total / samples);
}

float battery::ReadADCVoltage(uint8_t samples)
{
    if (samples == 0)
    {
        samples = 1;
    }

    uint32_t totalMillivolts = 0;

    // analogReadMilliVolts() uses the ESP32 Arduino ADC calibration path and is
    // preferable here to converting raw counts with an assumed 3.3V reference.
    for (uint8_t i = 0; i < samples; i++)
    {
        totalMillivolts += analogReadMilliVolts(BATTERY_SENSE_PIN);
        delay(1);
    }

    return (totalMillivolts / static_cast<float>(samples)) / 1000.0f;
}

float battery::ReadVoltage(uint8_t samples)
{
    return ReadADCVoltage(samples) * BATTERY_DIVIDER_GAIN;
}

void battery::SetEffectiveResistance(float resistanceOhms)
{
    if (resistanceOhms > 0.0f)
    {
        _effectiveResistanceOhms = resistanceOhms;
    }
    else
    {
        // Zero disables current estimation instead of silently using a made-up
        // battery resistance.
        _effectiveResistanceOhms = 0.0f;
    }
}

float battery::GetEffectiveResistance()
{
    return _effectiveResistanceOhms;
}

float battery::CaptureBaseline(uint8_t samples)
{
    _baselineVoltage = ReadVoltage(samples);
    _baselineValid = true;
    return _baselineVoltage;
}

float battery::GetBaselineVoltage()
{
    if (!_baselineValid)
    {
        return NAN;
    }

    return _baselineVoltage;
}

bool battery::HasBaseline()
{
    return _baselineValid;
}

void battery::ClearBaseline()
{
    _baselineVoltage = 0.0f;
    _baselineValid = false;
}

float battery::EstimateCurrent(uint8_t samples, float baselineCurrentA)
{
    if (!_baselineValid || _effectiveResistanceOhms <= 0.0f)
    {
        return NAN;
    }

    float loadedVoltage = ReadVoltage(samples);
    return EstimateCurrentFromVoltage(loadedVoltage, baselineCurrentA);
}

float battery::EstimateCurrentFromVoltage(float loadedVoltage,
                                          float baselineCurrentA)
{
    if (!_baselineValid)
    {
        return NAN;
    }

    return EstimateCurrentFromVoltages(_baselineVoltage,
                                       loadedVoltage,
                                       baselineCurrentA);
}

float battery::EstimateCurrentFromVoltages(float baselineVoltage,
                                           float loadedVoltage,
                                           float baselineCurrentA)
{
    if (_effectiveResistanceOhms <= 0.0f)
    {
        return NAN;
    }

    // From:
    //   Vbaseline = Voc - Ibaseline * Reffective
    //   Vload     = Voc - Iload     * Reffective
    // Therefore:
    //   Iload = Ibaseline + (Vbaseline - Vload) / Reffective
    return baselineCurrentA +
           ((baselineVoltage - loadedVoltage) / _effectiveResistanceOhms);
}

void battery::SetCellCount(uint8_t cells)
{
    if (cells > 0)
    {
        _cellCount = cells;
    }
}

uint8_t battery::GetCellCount()
{
    return _cellCount;
}

float battery::EstimateSOC(float packVoltage)
{
    if (_cellCount == 0 || packVoltage <= 0.0f)
    {
        return NAN;
    }

    float cellVoltage = packVoltage / static_cast<float>(_cellCount);

    if (cellVoltage >= LIPO_SOC_TABLE[0].voltsPerCell)
    {
        return 100.0f;
    }

    if (cellVoltage <= LIPO_SOC_TABLE[LIPO_SOC_TABLE_SIZE - 1].voltsPerCell)
    {
        return 0.0f;
    }

    // Linear interpolation between adjacent voltage/SOC points.
    for (size_t i = 0; i < LIPO_SOC_TABLE_SIZE - 1; i++)
    {
        const LiPoSOCPoint &high = LIPO_SOC_TABLE[i];
        const LiPoSOCPoint &low = LIPO_SOC_TABLE[i + 1];

        if (cellVoltage <= high.voltsPerCell &&
            cellVoltage >= low.voltsPerCell)
        {
            float fraction = (cellVoltage - low.voltsPerCell) /
                             (high.voltsPerCell - low.voltsPerCell);

            return low.percent + fraction * (high.percent - low.percent);
        }
    }

    return NAN;
}

float battery::EstimateSOCFromBaseline()
{
    if (!_baselineValid)
    {
        return NAN;
    }

    return EstimateSOC(_baselineVoltage);
}

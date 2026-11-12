#include "Termistores.h"
#include "Setup.h"

#include <Arduino.h>
#include <math.h>

// Conversión mediante divisor resistivo y modelo beta del NTC.
// Se sustituye el ajuste anterior, que no producía temperaturas plausibles.
static bool readTemperature(uint8_t pin, float &temperature_C)
{
    uint32_t totalMillivolts = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        totalMillivolts += analogReadMilliVolts(pin);
    }
    const float voltage = totalMillivolts / 4000.0f;
    temperature_C = 0.0f;

    // Rechazar valores próximos a los rieles: posible corto o circuito abierto.
    if (!isfinite(voltage) || voltage <= 0.05f || voltage >= ADC_MAX_VOLTAGE - 0.25f) {
        return false;
    }

    const float resistance = THERM_NTC_TO_GND
        ? THERM_FIXED_OHM * voltage / (ADC_MAX_VOLTAGE - voltage)
        : THERM_FIXED_OHM * (ADC_MAX_VOLTAGE - voltage) / voltage;
    const float inverseTemperature = 1.0f / THERM_T0_K +
                                    logf(resistance / THERM_R25_OHM) / THERM_BETA_K;
    if (!isfinite(inverseTemperature) || inverseTemperature <= 0.0f) {
        return false;
    }
    const float result = 1.0f / inverseTemperature - 273.15f;
    if (!isfinite(result) || result < -40.0f || result > 150.0f) {
        return false;
    }
    temperature_C = result;
    return true;
}

void beginTermistores()
{
    pinMode(THERM_1_PIN, INPUT);
    pinMode(THERM_2_PIN, INPUT);
    pinMode(THERM_3_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(THERM_1_PIN, ADC_11db);
    analogSetPinAttenuation(THERM_2_PIN, ADC_11db);
    analogSetPinAttenuation(THERM_3_PIN, ADC_11db);
}

void updateTermistores(Telemetria &telemetry)
{
    // Cada temperatura tiene su propia bandera; una falla no invalida las otras.
    telemetry.therm1Ok = readTemperature(THERM_1_PIN, telemetry.therm1_C);
    telemetry.therm2Ok = readTemperature(THERM_2_PIN, telemetry.therm2_C);
    telemetry.therm3Ok = readTemperature(THERM_3_PIN, telemetry.therm3_C);
}

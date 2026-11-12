#include "Packet.h"

#include <math.h>
#include <string.h>

// Conversión explícita para no depender de alineación, padding ni endianness.
static void write16(uint8_t *buffer, size_t offset, uint16_t value)
{
    buffer[offset] = static_cast<uint8_t>(value);
    buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
}

static void write32(uint8_t *buffer, size_t offset, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i) {
        buffer[offset + i] = static_cast<uint8_t>(value >> (8 * i));
    }
}

// Un valor fuera de rango se invalida; no se permite que el entero desborde.
static bool fits(double value, double scale, double minimum, double maximum)
{
    if (!isfinite(value)) {
        return false;
    }
    const double encoded = round(value * scale);
    return isfinite(encoded) && encoded >= minimum && encoded <= maximum;
}

static bool fitsVector16(const float *values, double scale)
{
    for (size_t i = 0; i < 3; ++i) {
        if (!fits(values[i], scale, -32768.0, 32767.0)) {
            return false;
        }
    }
    return true;
}

static void writeSigned16(uint8_t *buffer, size_t offset, double value, double scale)
{
    write16(buffer, offset, static_cast<uint16_t>(static_cast<int16_t>(round(value * scale))));
}

static void writeSigned32(uint8_t *buffer, size_t offset, double value, double scale)
{
    write32(buffer, offset, static_cast<uint32_t>(static_cast<int32_t>(round(value * scale))));
}

size_t buildPacket(const Telemetria &telemetry, uint8_t *buffer, size_t capacity)
{
    if (buffer == nullptr || capacity < PACKET_LEN) {
        return 0;
    }

    // Los datos inválidos y los dos bytes reservados quedan en cero.
    memset(buffer, 0, PACKET_LEN);
    buffer[0] = telemetry.teamId;
    buffer[1] = telemetry.sequence;
    uint8_t status = 0;

    if (telemetry.bmsOk && telemetry.voltage_V >= 0.0f &&
        fits(telemetry.voltage_V, 100.0, 0.0, 65535.0) &&
        fits(telemetry.current_A, 100.0, -32768.0, 32767.0)) {
        status |= STATUS_BMS;
        write16(buffer, 3, static_cast<uint16_t>(round(telemetry.voltage_V * 100.0)));
        writeSigned16(buffer, 5, telemetry.current_A, 100.0);
    }

    bool imuValid = telemetry.mpuOk &&
                    fitsVector16(telemetry.angularRate_dps, 100.0);
    if (telemetry.imuCalibrated) {
        imuValid = imuValid && fitsVector16(telemetry.velocity_mps, 100.0) &&
                   fitsVector16(telemetry.acceleration_mps2, 100.0);
        for (size_t i = 0; i < 3; ++i) {
            imuValid = imuValid && fits(telemetry.position_m[i], 1000.0,
                                       -2147483648.0, 2147483647.0);
        }
    }

    if (imuValid) {
        status |= STATUS_MPU;
        for (size_t i = 0; i < 3; ++i) {
            writeSigned16(buffer, 31 + 2 * i, telemetry.angularRate_dps[i], 100.0);
        }
        // Antes de calibrar, velocidad, posición y aceleración quedan en cero.
        if (telemetry.imuCalibrated) {
            status |= STATUS_IMU_CALIBRATED;
            for (size_t i = 0; i < 3; ++i) {
                writeSigned16(buffer, 7 + 2 * i, telemetry.velocity_mps[i], 100.0);
                writeSigned32(buffer, 13 + 4 * i, telemetry.position_m[i], 1000.0);
                writeSigned16(buffer, 25 + 2 * i, telemetry.acceleration_mps2[i], 100.0);
            }
        }
    }

    const float temperatures[] = {telemetry.therm1_C, telemetry.therm2_C, telemetry.therm3_C};
    const bool validTemperatures[] = {telemetry.therm1Ok, telemetry.therm2Ok, telemetry.therm3Ok};
    for (size_t i = 0; i < 3; ++i) {
        if (validTemperatures[i] && fits(temperatures[i], 10.0, -32768.0, 32767.0)) {
            status |= static_cast<uint8_t>(STATUS_NTC1 << i);
            writeSigned16(buffer, 37 + 2 * i, temperatures[i], 10.0);
        }
    }

    if (telemetry.gpsValid && isfinite(telemetry.latitude_deg) &&
        isfinite(telemetry.longitude_deg) &&
        fabs(telemetry.latitude_deg) <= 90.0 && fabs(telemetry.longitude_deg) <= 180.0 &&
        fits(telemetry.altitude_m, 100.0, -2147483648.0, 2147483647.0)) {
        status |= STATUS_GPS;
        writeSigned32(buffer, 43, telemetry.latitude_deg, 1.0e7);
        writeSigned32(buffer, 47, telemetry.longitude_deg, 1.0e7);
        writeSigned32(buffer, 51, telemetry.altitude_m, 100.0);
    }

    buffer[2] = status;
    return PACKET_LEN;
}

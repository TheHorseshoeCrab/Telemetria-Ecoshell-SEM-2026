#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include "Setup.h"

// Se conserva el nombre de la estructura original.
// Esta estructura vive en RAM: NO se transmite directamente con sizeof().
struct Telemetria
{
    uint8_t teamId = TEAM_ID;
    uint8_t sequence = 0;

    float voltage_V = 0.0f;
    float current_A = 0.0f;
    // El porcentaje se usa solo en el display; no ocupa bytes del paquete.
    uint8_t batteryPercent = 0;
    bool batteryPercentValid = false;

    // Velocidad y posición en el marco inicial, con origen al calibrar.
    float velocity_mps[3] = {};
    float position_m[3] = {};
    // Aceleración sin gravedad en el marco inicial; gyro en marco vehículo.
    float acceleration_mps2[3] = {};
    float angularRate_dps[3] = {};

    float therm1_C = 0.0f;
    float therm2_C = 0.0f;
    float therm3_C = 0.0f;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    float altitude_m = 0.0f;
    // Velocidad sobre el suelo del GPS: solo para el display, no va en el paquete.
    float gpsSpeed_kmh = 0.0f;
    bool gpsSpeedValid = false;

    bool bmsOk = false;
    bool mpuOk = false;
    bool gpsValid = false;
    bool therm1Ok = false;
    bool therm2Ok = false;
    bool therm3Ok = false;
    bool imuCalibrated = false;
};

// El estado se construye a partir de las mediciones válidas al empaquetar.
enum TelemetryStatus : uint8_t
{
    STATUS_BMS = 1U << 0,
    STATUS_MPU = 1U << 1,
    STATUS_GPS = 1U << 2,
    STATUS_NTC1 = 1U << 3,
    STATUS_NTC2 = 1U << 4,
    STATUS_NTC3 = 1U << 5,
    STATUS_IMU_CALIBRATED = 1U << 6
};

#endif

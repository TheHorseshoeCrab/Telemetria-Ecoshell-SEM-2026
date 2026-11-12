#include "GPS.h"
#include "Setup.h"

#include <Arduino.h>
#include <TinyGPS++.h>
#include <math.h>

static TinyGPSPlus gps;
static HardwareSerial gpsSerial(1);

void beginGPS()
{
    gpsSerial.setRxBufferSize(1024);
    gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void updateGPS(Telemetria &telemetry)
{
    // Con los demás módulos no bloqueantes, basta atender la UART en el loop.
    // Así evitamos una segunda tarea y accesos concurrentes a la estructura.
    for (size_t budget = 0; budget < 128 && gpsSerial.available() > 0; ++budget) {
        gps.encode(static_cast<char>(gpsSerial.read()));
    }

    // Velocidad del RMC: solo con fix y si es reciente (el GPS la da a ~1 Hz).
    const double speed = gps.speed.kmph();
    telemetry.gpsSpeedValid = gps.speed.isValid() && gps.location.isValid() &&
                              gps.speed.age() < GPS_SPEED_TIMEOUT_MS &&
                              gps.location.age() < GPS_SPEED_TIMEOUT_MS &&
                              isfinite(speed) && speed >= 0.0 && speed < 500.0;
    telemetry.gpsSpeed_kmh = telemetry.gpsSpeedValid ? static_cast<float>(speed) : 0.0f;

    const bool valid = gps.location.isValid() && gps.altitude.isValid() &&
                       gps.location.age() < GPS_DATA_TIMEOUT_MS &&
                       gps.altitude.age() < GPS_DATA_TIMEOUT_MS;
    if (valid) {
        const double latitude = gps.location.lat();
        const double longitude = gps.location.lng();
        const double altitude = gps.altitude.meters();
        telemetry.gpsValid = isfinite(latitude) && isfinite(longitude) && isfinite(altitude) &&
                             fabs(latitude) <= 90.0 && fabs(longitude) <= 180.0;
        if (telemetry.gpsValid) {
            telemetry.latitude_deg = latitude;
            telemetry.longitude_deg = longitude;
            telemetry.altitude_m = static_cast<float>(altitude);
            return;
        }
    }

    // Una sola bandera valida el grupo completo latitud/longitud/altitud.
    telemetry.gpsValid = false;
    telemetry.latitude_deg = telemetry.longitude_deg = 0.0;
    telemetry.altitude_m = 0.0f;
}

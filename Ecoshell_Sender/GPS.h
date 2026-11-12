#ifndef GPS_H
#define GPS_H

#include "Telemetria.h"

// Procesa NMEA sin sustituir la velocidad estimada por la IMU.
void beginGPS();
void updateGPS(Telemetria &telemetry);

#endif

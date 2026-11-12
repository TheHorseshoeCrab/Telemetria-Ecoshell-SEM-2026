#ifndef MPU6050_MODULE_H
#define MPU6050_MODULE_H

#include "Telemetria.h"

// Configura el sensor y procesa una muestra nueva cuando esté disponible.
bool beginMPU6050();
void updateMPU6050(Telemetria &telemetry);
// Diagnóstico: mayor intervalo entre muestras desde la última consulta (µs).
uint32_t takeImuMaxIntervalUs();

#endif

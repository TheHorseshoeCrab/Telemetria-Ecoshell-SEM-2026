#ifndef PACKET_H
#define PACKET_H

#include "Telemetria.h"

// Escribe exactamente 57 bytes, sin padding y en little-endian.
// Devuelve cero si el búfer no tiene capacidad suficiente.
size_t buildPacket(const Telemetria &telemetry, uint8_t *buffer, size_t capacity);

#endif

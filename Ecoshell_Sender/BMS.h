#ifndef BMS_H
#define BMS_H

#include "Telemetria.h"

// Inicialización UART y servicio no bloqueante del protocolo JBD.
void beginBMS();
void updateBMS(Telemetria &telemetry);

#endif

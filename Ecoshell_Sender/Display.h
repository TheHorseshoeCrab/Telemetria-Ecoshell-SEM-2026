#ifndef DISPLAY_H
#define DISPLAY_H

#include "Telemetria.h"

// Inicializa el dashboard y actualiza sus regiones de forma escalonada.
void beginDisplay();
void updateDisplay(const Telemetria &telemetry, bool radioReady, bool radioBusy);

#endif

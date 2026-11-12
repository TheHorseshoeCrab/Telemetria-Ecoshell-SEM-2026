#ifndef ECOSHELL_LORA_H
#define ECOSHELL_LORA_H

#include "Telemetria.h"

// Se usa RadioLib internamente para evitar la colisión Lora.h/LoRa.h en Windows.
bool beginLora();
void serviceLora();
bool sendTelemetry(const Telemetria &telemetry);
bool isLoraReady();
bool isLoraBusy();
int16_t getLoraError();

#endif

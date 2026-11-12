#include "Lora.h"
#include "Packet.h"
#include "Setup.h"

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// RadioLib permite conservar Lora.h sin colisionar con otra librería LoRa.h.
static Module radioModule(LORA_SS_PIN, LORA_DIO0_PIN,
                          LORA_RST_PIN < 0 ? RADIOLIB_NC : static_cast<uint32_t>(LORA_RST_PIN),
                          RADIOLIB_NC, SPI, SPISettings(8000000, MSBFIRST, SPI_MODE0));
static SX1278 radio(&radioModule);
static volatile bool transmissionDone = false;
static bool radioInitialized = false;  // begin() correcto al arrancar
static bool transmitting = false;
static int16_t lastError = RADIOLIB_ERR_NONE;
static uint32_t transmissionStartedMs = 0;
static uint8_t packet[PACKET_LEN];

static void IRAM_ATTR onTransmissionDone()
{
    // La interrupción solo levanta una bandera; no accede al bus SPI.
    transmissionDone = true;
}

bool beginLora()
{
    lastError = radio.begin(LORA_FREQ_HZ / 1.0e6f, LORA_BANDWIDTH_KHZ,
                            LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                            LORA_SYNC_WORD, LORA_TX_POWER_DBM, LORA_PREAMBLE_SYMBOLS);
    if (lastError == RADIOLIB_ERR_NONE) {
        // CRC físico del módem activado; no se añade un CRC de software al payload.
        lastError = radio.setCRC(true);
    }
    radioInitialized = lastError == RADIOLIB_ERR_NONE;
    if (radioInitialized) {
        radio.setDio0Action(onTransmissionDone, RISING);
    }
    transmitting = transmissionDone = false;
    return radioInitialized;
}

// Un fallo de transmisión no apaga la radio: se regresa a standby y se reintenta
// en el siguiente periodo. No se repite begin() en el loop porque, sin el chip,
// puede bloquear ~100 ms y provocar una pausa que reinicia la estimación IMU.
void serviceLora()
{
    if (!radioInitialized || !transmitting) {
        return;
    }
    if (transmissionDone) {
        transmissionDone = false;
        lastError = radio.finishTransmit();
        transmitting = false;
    } else if (millis() - transmissionStartedMs >= LORA_TX_TIMEOUT_MS) {
        // Sin TxDone en DIO0: revisar el cableado de DIO0 si se repite.
        radio.finishTransmit();
        transmitting = false;
        lastError = RADIOLIB_ERR_TX_TIMEOUT;
    }
}

bool sendTelemetry(const Telemetria &telemetry)
{
    if (!radioInitialized || transmitting ||
        buildPacket(telemetry, packet, sizeof(packet)) != PACKET_LEN) {
        return false;
    }
    transmissionDone = false;
    const int16_t state = radio.startTransmit(packet, PACKET_LEN);
    if (state != RADIOLIB_ERR_NONE) {
        lastError = state;
        radio.standby();
        return false;
    }
    transmissionStartedMs = millis();
    transmitting = true;
    return true;
}

// "Listo" = inicializada y sin error en la última transmisión.
bool isLoraReady() { return radioInitialized && lastError == RADIOLIB_ERR_NONE; }
bool isLoraBusy() { return transmitting; }
int16_t getLoraError() { return lastError; }

#include "BMS.h"
#include "Setup.h"
#include <Arduino.h>

static HardwareSerial bmsSerial(2);
static constexpr uint32_t BMS_READ_INTERVAL_MS = 1000;
// Algunos JBD tardan >150 ms en responder; la ventana cabe holgada en 1 s.
static constexpr uint32_t BMS_RESPONSE_TIMEOUT_MS = 400;
static constexpr uint32_t BMS_DATA_TIMEOUT_MS = 2500;
static constexpr size_t BMS_FRAME_MAX_LEN = 80;
static const uint8_t JBD_READ_BASIC_INFO_CMD[] = {
    0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77
};

static uint8_t frame[BMS_FRAME_MAX_LEN];
static size_t receivedLength = 0;
static size_t expectedLength = 0;
static uint32_t lastBmsReadMs = 0;
static uint32_t requestStartedMs = 0;
static uint32_t lastValidMs = 0;
static bool waitingForResponse = false;
static bool hasValidData = false;

static uint16_t readUInt16BE(const uint8_t *data)
{
    return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

static int16_t readInt16BE(const uint8_t *data)
{
    const uint16_t value = readUInt16BE(data);
    const int32_t signedValue = (value & 0x8000U) ? static_cast<int32_t>(value) - 65536 : value;
    return static_cast<int16_t>(signedValue);
}

static void invalidateBMS(Telemetria &telemetry)
{
    telemetry.bmsOk = false;
    telemetry.batteryPercentValid = false;
    telemetry.voltage_V = 0.0f;
    telemetry.current_A = 0.0f;
    telemetry.batteryPercent = 0;
}

static bool decodeFrame(Telemetria &telemetry)
{
    const size_t dataLength = frame[3];
    if (frame[1] != 0x03 || frame[2] != 0 || dataLength < 4 ||
        frame[expectedLength - 1] != 0x77) {
        return false;
    }

    // El checksum JBD se conserva; es independiente del paquete LoRa.
    uint16_t sum = 0;
    for (size_t i = 2; i < 4 + dataLength; ++i) {
        sum = static_cast<uint16_t>(sum + frame[i]);
    }
    const uint16_t checksum = static_cast<uint16_t>(0U - sum);
    if (checksum != readUInt16BE(&frame[4 + dataLength])) {
        return false;
    }

    const uint8_t *data = &frame[4];
    telemetry.voltage_V = readUInt16BE(data) / 100.0f;
    telemetry.current_A = readInt16BE(data + 2) / 100.0f;
    telemetry.bmsOk = true;
    // RSOC ocupa el índice 19 del payload de información básica JBD.
    telemetry.batteryPercentValid = dataLength >= 20 && data[19] <= 100;
    telemetry.batteryPercent = telemetry.batteryPercentValid ? data[19] : 0;
    return true;
}

void beginBMS()
{
    bmsSerial.setRxBufferSize(256);
    bmsSerial.begin(BMS_BAUD_RATE, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
    receivedLength = expectedLength = 0;
    waitingForResponse = hasValidData = false;
    lastBmsReadMs = millis() - BMS_READ_INTERVAL_MS;
}

void updateBMS(Telemetria &telemetry)
{
    const uint32_t now = millis();
    if (!hasValidData || now - lastValidMs > BMS_DATA_TIMEOUT_MS) {
        invalidateBMS(telemetry);
    }

    // Un único plazo cubre toda la respuesta, sin esperar byte por byte.
    // Una respuesta perdida o dañada no invalida de inmediato: se conserva el
    // último dato verificado hasta BMS_DATA_TIMEOUT_MS (2 consultas fallidas).
    if (waitingForResponse && now - requestStartedMs >= BMS_RESPONSE_TIMEOUT_MS) {
        waitingForResponse = false;
    }

    // Trabajo acotado por vuelta para no monopolizar el loop ante ruido UART.
    for (size_t budget = 0; budget < BMS_FRAME_MAX_LEN && bmsSerial.available() > 0; ++budget) {
        const uint8_t value = static_cast<uint8_t>(bmsSerial.read());
        if (!waitingForResponse) {
            continue;
        }
        if (receivedLength == 0 && value != 0xDD) {
            continue;
        }
        frame[receivedLength++] = value;
        if (receivedLength == 4) {
            expectedLength = static_cast<size_t>(frame[3]) + 7;
            if (expectedLength > BMS_FRAME_MAX_LEN) {
                waitingForResponse = false;
                continue;
            }
        }
        if (expectedLength != 0 && receivedLength == expectedLength) {
            waitingForResponse = false;
            // decodeFrame solo escribe en telemetry si la trama es correcta.
            if (decodeFrame(telemetry)) {
                lastValidMs = now;
                hasValidData = true;
            }
        }
    }

    if (!waitingForResponse && now - lastBmsReadMs >= BMS_READ_INTERVAL_MS &&
        bmsSerial.available() == 0 &&
        bmsSerial.availableForWrite() >= static_cast<int>(sizeof(JBD_READ_BASIC_INFO_CMD))) {
        receivedLength = expectedLength = 0;
        lastBmsReadMs = requestStartedMs = now;
        waitingForResponse = true;
        bmsSerial.write(JBD_READ_BASIC_INFO_CMD, sizeof(JBD_READ_BASIC_INFO_CMD));
        // Sin flush(): la UART termina de transmitir en segundo plano.
    }
}

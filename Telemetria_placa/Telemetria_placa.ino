#include <Arduino.h>
#include <SPI.h>

#include "Setup.h"
#include "Telemetria.h"
#include "Termistores.h"
#include "GPS.h"
#include "BMS.h"
#include "MPU6050.h"
#include "Lora.h"
#include "Display.h"

static Telemetria telemetry;
static uint32_t lastTemperatureMs = 0;
static uint32_t lastTransmissionMs = 0;
static uint32_t lastDebugPrintMs = 0;

// Prueba: diagnóstico resumido, opcional y sin esperar a que se conecte USB.
static void printTelemetry()
{
    if (!DEBUG_ENABLED || !Serial) {
        return;
    }
    // IMU_DT_MAX: mayor intervalo entre muestras IMU en el último segundo.
    // Debe quedar cerca de 10000 us; valores ~20000 indican muestras perdidas.
    char message[160];
    const int length = snprintf(message, sizeof(message),
        "SEQ=%u BMS=%u GPS=%u MPU=%u CAL=%u NTC=%u%u%u LORA=%d IMU_DT_MAX=%lu us\n",
        telemetry.sequence, telemetry.bmsOk, telemetry.gpsValid, telemetry.mpuOk,
        telemetry.imuCalibrated, telemetry.therm1Ok, telemetry.therm2Ok,
        telemetry.therm3Ok, getLoraError(),
        static_cast<unsigned long>(takeImuMaxIntervalUs()));
    if (length > 0 && length < static_cast<int>(sizeof(message)) &&
        Serial.availableForWrite() >= length) {
        Serial.write(reinterpret_cast<const uint8_t *>(message), length);
    }
}

// Inicialización: cada módulo es responsable de su periférico.
void setup()
{
    Serial.begin(DEBUG_BAUD_RATE);

    // Desactivar ambos dispositivos antes de inicializar el SPI compartido.
    pinMode(LORA_SS_PIN, OUTPUT);
    pinMode(TFT_CS_PIN, OUTPUT);
    digitalWrite(LORA_SS_PIN, HIGH);
    digitalWrite(TFT_CS_PIN, HIGH);
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN);

    beginGPS();
    beginBMS();
    beginTermistores();
    beginDisplay();
    beginLora();
    // Inicializar la IMU al final: la calibración se realiza después en loop().
    beginMPU6050();

    updateTermistores(telemetry);
    const uint32_t now = millis();
    lastTemperatureMs = lastTransmissionMs = lastDebugPrintMs = now;
}

// Loop: atender los sensores y distribuir sus datos a LoRa y al display.
void loop()
{
    // El MPU usa su bandera DATA_RDY, con adquisición configurada a 100 Hz.
    updateMPU6050(telemetry);
    updateGPS(telemetry);
    updateBMS(telemetry);
    serviceLora();

    const uint32_t now = millis();
    if (now - lastTemperatureMs >= THERM_READ_INTERVAL_MS) {
        lastTemperatureMs = now;
        updateTermistores(telemetry);
    }
    if (now - lastTransmissionMs >= LORA_SEND_INTERVAL_MS) {
        lastTransmissionMs = now;
        if (sendTelemetry(telemetry)) {
            // Incrementar al aceptar el envío; uint8_t regresa a cero tras 255.
            ++telemetry.sequence;
        }
    }

    // La función controla el periodo de 200 ms y dibuja una región por vuelta.
    updateDisplay(telemetry, isLoraReady(), isLoraBusy());

    if (DEBUG_ENABLED && now - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
        lastDebugPrintMs = now;
        printTelemetry();
    }
    // Ceder CPU al sistema; no usar delay(1000) para programar las tareas.
    delay(1);
}

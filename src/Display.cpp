#include "Display.h"
#include "Setup.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <math.h>
#include <stdio.h>

static Adafruit_ILI9341 display(&SPI, TFT_DC_PIN, TFT_CS_PIN, TFT_RST_PIN);
static constexpr uint16_t BACKGROUND = 0x3186;
static constexpr uint16_t FOREGROUND = ILI9341_WHITE;
static uint32_t lastRefreshMs = 0;
static uint8_t region = 9;
static Telemetria snapshot;
static bool snapshotRadioReady = false;
static bool snapshotRadioBusy = false;
enum SpeedSource : int8_t { SPEED_UNKNOWN = -1, SPEED_NONE = 0, SPEED_GPS, SPEED_IMU };
static SpeedSource shownSpeedSource = SPEED_UNKNOWN;

static void drawText(int16_t x, int16_t y, uint8_t size, const char *text)
{
    display.setTextColor(FOREGROUND, BACKGROUND);
    display.setTextSize(size);
    display.setCursor(x, y);
    display.print(text);
}

void beginDisplay()
{
    display.begin(TFT_SPI_FREQUENCY_HZ);
    display.setRotation(1);
    display.setTextWrap(false);
    display.fillScreen(BACKGROUND);
    drawText(12, 8, 2, "SEM 2026");
    drawText(172, 12, 1, "ECOSHELL RACING");
    display.drawFastHLine(12, 29, 296, FOREGROUND);
    shownSpeedSource = SPEED_UNKNOWN;  // La etiqueta se dibuja en la primera región 0
    lastRefreshMs = millis() - DISPLAY_UPDATE_INTERVAL_MS;
    region = 9;
}

void updateDisplay(const Telemetria &telemetry, bool radioReady, bool radioBusy)
{
    const uint32_t now = millis();
    if (region >= 9) {
        if (now - lastRefreshMs < DISPLAY_UPDATE_INTERVAL_MS) {
            return;
        }
        lastRefreshMs = now;
        snapshot = telemetry;
        snapshotRadioReady = radioReady;
        snapshotRadioBusy = radioBusy;
        region = 0;
    }

    // Una región por llamada reduce las pausas de adquisición por tráfico SPI.
    char text[48];
    switch (region++) {
    case 0: {
        display.fillRect(12, 38, 196, 34, BACKGROUND);
        // Fuente preferida: GPS (no deriva). Respaldo: estimación IMU horizontal,
        // que acumula error con el tiempo. La etiqueta indica cuál se muestra.
        const float imuSpeed = sqrtf(snapshot.velocity_mps[0]*snapshot.velocity_mps[0] +
                                     snapshot.velocity_mps[1]*snapshot.velocity_mps[1]) * 3.6f;
        SpeedSource source = SPEED_NONE;
        float speed = 0.0f;
        if (snapshot.gpsSpeedValid) {
            source = SPEED_GPS;
            speed = snapshot.gpsSpeed_kmh;
        } else if (snapshot.mpuOk && snapshot.imuCalibrated && isfinite(imuSpeed) &&
                   imuSpeed < 1000.0f) {
            source = SPEED_IMU;
            speed = imuSpeed;
        }
        if (source != SPEED_NONE) {
            snprintf(text, sizeof(text), "%5.1f", static_cast<double>(speed));
        } else {
            snprintf(text, sizeof(text), "  --");
        }
        drawText(16, 40, 4, text);
        // Redibujar la etiqueta solo si cambia la fuente.
        if (source != shownSpeedSource) {
            shownSpeedSource = source;
            display.fillRect(12, 76, 196, 10, BACKGROUND);
            drawText(16, 78, 1, source == SPEED_GPS ? "km/h  |  GPS" :
                                source == SPEED_IMU ? "km/h  |  ESTIMACION IMU" :
                                                      "km/h  |  SIN DATO");
        }
        break;
    }
    case 1: {
        display.fillRect(226, 38, 82, 52, BACKGROUND);
        display.drawRect(228, 40, 70, 25, FOREGROUND);
        display.fillRect(298, 47, 4, 11, FOREGROUND);
        const bool valid = snapshot.bmsOk && snapshot.batteryPercentValid;
        for (uint8_t i = 0; i < 4; ++i) {
            const bool filled = valid && snapshot.batteryPercent > i * 25;
            display.fillRect(232 + i*16, 44, 13, 17,
                             filled ? ILI9341_GREEN : ILI9341_DARKGREY);
        }
        if (valid) {
            snprintf(text, sizeof(text), "%3u %%", snapshot.batteryPercent);
        } else {
            snprintf(text, sizeof(text), "N/C");
        }
        drawText(238, 74, 1, text);
        break;
    }
    case 2:
        display.fillRect(12, 97, 296, 18, BACKGROUND);
        if (snapshot.bmsOk) {
            snprintf(text, sizeof(text), "%5.2f V   %+6.2f A",
                     static_cast<double>(snapshot.voltage_V), static_cast<double>(snapshot.current_A));
        } else {
            snprintf(text, sizeof(text), "  -- V       -- A");
        }
        drawText(12, 98, 2, text);
        break;
    case 3:
        display.fillRect(12, 119, 296, 16, BACKGROUND);
        if (snapshot.bmsOk) {
            snprintf(text, sizeof(text), "Power: %.1f W",
                     static_cast<double>(snapshot.voltage_V * snapshot.current_A));
        } else {
            snprintf(text, sizeof(text), "Power: -- W");
        }
        drawText(12, 120, 1, text);
        break;
    case 4:
    case 5:
    case 6: {
        const uint8_t index = region - 5;
        const float values[] = {snapshot.therm1_C, snapshot.therm2_C, snapshot.therm3_C};
        const bool valid[] = {snapshot.therm1Ok, snapshot.therm2Ok, snapshot.therm3Ok};
        const int16_t y = 140 + index * 21;
        display.fillRect(12, y, 296, 18, BACKGROUND);
        if (valid[index]) {
            snprintf(text, sizeof(text), "T%u: %6.1f C", index + 1, static_cast<double>(values[index]));
        } else {
            snprintf(text, sizeof(text), "T%u:     -- C", index + 1);
        }
        drawText(12, y, 2, text);
        break;
    }
    case 7:
        display.fillRect(12, 208, 296, 10, BACKGROUND);
        snprintf(text, sizeof(text), "BMS:%s GPS:%s IMU:%s", snapshot.bmsOk ? "OK" : "--",
                 snapshot.gpsValid ? "OK" : "--",
                 !snapshot.mpuOk ? "--" : snapshot.imuCalibrated ? "OK" : "CAL");
        drawText(12, 208, 1, text);
        break;
    case 8:
        display.fillRect(12, 224, 296, 10, BACKGROUND);
        snprintf(text, sizeof(text), "LoRa:%s  SEQ:%u", !snapshotRadioReady ? "ERROR" :
                 snapshotRadioBusy ? "TX" : "READY", snapshot.sequence);
        drawText(12, 224, 1, text);
        break;
    }
}

#ifndef SETUP_H
#define SETUP_H

#include <stdint.h>
#include <stddef.h>

static constexpr float ADC_MAX_VALUE = 4095.0f;  // Resolución de 12 bits
static constexpr float ADC_MAX_VOLTAGE = 3.3f;   // Alimentación nominal del divisor
static constexpr uint8_t TEAM_ID = 11;
static constexpr uint8_t PACKET_LEN = 57;

// LoRa: estos parámetros también deben coincidir en la base receptora.
static constexpr long LORA_FREQ_HZ = 433000000L;
static constexpr uint8_t LORA_TX_POWER_DBM = 17;
static constexpr float LORA_BANDWIDTH_KHZ = 125.0f;
static constexpr uint8_t LORA_SPREADING_FACTOR = 7;
static constexpr uint8_t LORA_CODING_RATE = 5;  // 4/5
static constexpr uint8_t LORA_SYNC_WORD = 0x12;
static constexpr uint16_t LORA_PREAMBLE_SYMBOLS = 8;
static constexpr uint8_t LORA_SCK_PIN = 12;
static constexpr uint8_t LORA_MISO_PIN = 13;
static constexpr uint8_t LORA_MOSI_PIN = 11;
static constexpr uint8_t LORA_SS_PIN = 7;
static constexpr int LORA_RST_PIN = -1;  // RST conectado a 3.3 V
static constexpr int LORA_DIO0_PIN = 16;
static constexpr uint32_t LORA_SEND_INTERVAL_MS = 1000;
static constexpr uint32_t LORA_TX_TIMEOUT_MS = 900;

// Display ILI9341: configuración de la demo, SPI compartido con LoRa.
static constexpr uint8_t TFT_CS_PIN = 10;
static constexpr uint8_t TFT_DC_PIN = 15;
static constexpr uint8_t TFT_RST_PIN = 14;
static constexpr uint32_t TFT_SPI_FREQUENCY_HZ = 20000000;
static constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 200;

// MPU6050 horizontal. AD0 bajo: dirección 0x68.
static constexpr uint8_t MPU_SDA_PIN = 8;
static constexpr uint8_t MPU_SCL_PIN = 9;
static constexpr uint8_t MPU_I2C_ADDRESS = 0x68;
static constexpr uint32_t IMU_SAMPLE_INTERVAL_US = 10000;
static_assert(IMU_SAMPLE_INTERVAL_US >= 1000 && IMU_SAMPLE_INTERVAL_US <= 50000 &&
              IMU_SAMPLE_INTERVAL_US % 1000 == 0,
              "El periodo IMU debe ser multiplo de 1 ms, entre 1 y 50 ms");
static constexpr uint16_t IMU_CALIBRATION_SAMPLES = 500;
// Reposo durante calibración: el bias de fábrica del MPU6050 puede llegar a ±20 °/s
// (ZRO del datasheet), así que se limita el valor absoluto con margen y se exige
// que cada muestra se mantenga cerca del promedio acumulado (giro constante).
static constexpr float IMU_STILL_GYRO_MAX_DPS = 25.0f;
static constexpr float IMU_STILL_GYRO_DEVIATION_DPS = 3.0f;
// Sin datos nuevos durante este tiempo: sensor caído o estimación interrumpida.
static constexpr uint32_t IMU_MAX_GAP_US = 50000;
// Reintento de inicialización de un MPU caído (bloquea ~110 ms por intento).
static constexpr uint32_t IMU_REINIT_INTERVAL_MS = 2000;
static constexpr float GRAVITY_MPS2 = 9.80665f;
static constexpr float IMU_FILTER_TAU_S = 0.49f;  // Equivale a alfa 0.98 a 100 Hz

// Mapeo provisional sensor -> vehículo: +X adelante, +Y izquierda, +Z arriba.
// Montaje horizontal no determina el rumbo. Ajustar si el sensor está girado.
// Usar una permutación de 0,1,2 y signos que formen un sistema derecho.
static constexpr uint8_t IMU_AXIS_MAP[3] = {0, 1, 2};
static constexpr int8_t IMU_AXIS_SIGN[3] = {1, 1, 1};
// Evitar índices repetidos/fuera de rango al adaptar el montaje.
static_assert(IMU_AXIS_MAP[0] < 3 && IMU_AXIS_MAP[1] < 3 && IMU_AXIS_MAP[2] < 3,
              "IMU_AXIS_MAP debe usar indices entre 0 y 2");
static_assert(IMU_AXIS_MAP[0] != IMU_AXIS_MAP[1] && IMU_AXIS_MAP[0] != IMU_AXIS_MAP[2] &&
              IMU_AXIS_MAP[1] != IMU_AXIS_MAP[2], "IMU_AXIS_MAP no debe repetir ejes");
static_assert((IMU_AXIS_SIGN[0] == 1 || IMU_AXIS_SIGN[0] == -1) &&
              (IMU_AXIS_SIGN[1] == 1 || IMU_AXIS_SIGN[1] == -1) &&
              (IMU_AXIS_SIGN[2] == 1 || IMU_AXIS_SIGN[2] == -1),
              "IMU_AXIS_SIGN solo permite +1 o -1");
static constexpr int IMU_AXIS_PARITY =
    ((IMU_AXIS_MAP[0] > IMU_AXIS_MAP[1]) + (IMU_AXIS_MAP[0] > IMU_AXIS_MAP[2]) +
     (IMU_AXIS_MAP[1] > IMU_AXIS_MAP[2])) % 2 == 0 ? 1 : -1;
static_assert(IMU_AXIS_PARITY * IMU_AXIS_SIGN[0] * IMU_AXIS_SIGN[1] * IMU_AXIS_SIGN[2] == 1,
              "El mapeo IMU debe conservar un sistema de ejes derecho");

// Termistores: T1 detrás del firewall; T2 y T3 dentro de la cabina.
static constexpr uint8_t THERM_1_PIN = 4;
static constexpr uint8_t THERM_2_PIN = 5;
static constexpr uint8_t THERM_3_PIN = 6;
static constexpr uint32_t THERM_READ_INTERVAL_MS = 1000;
// Modelo beta: verificar estos valores con los componentes instalados.
static constexpr float THERM_R25_OHM = 10000.0f;
static constexpr float THERM_FIXED_OHM = 10000.0f;
static constexpr float THERM_BETA_K = 3950.0f;
static constexpr float THERM_T0_K = 298.15f;
static constexpr bool THERM_NTC_TO_GND = true;

// UART conservados exactamente como en los archivos originales.
// RX/TX corresponden al ESP32, no al periférico.
static constexpr uint8_t GPS_RX_PIN = 42;
static constexpr uint8_t GPS_TX_PIN = 41;
static constexpr uint32_t GPS_BAUD_RATE = 9600;
static constexpr uint32_t GPS_DATA_TIMEOUT_MS = 5000;
// La velocidad del display caduca antes que la posición: un dato viejo engaña más.
static constexpr uint32_t GPS_SPEED_TIMEOUT_MS = 2500;
static constexpr uint8_t BMS_RX_PIN = 17;
static constexpr uint8_t BMS_TX_PIN = 18;
static constexpr uint32_t BMS_BAUD_RATE = 9600;

// Diagnóstico USB desactivado durante la adquisición normal.
static constexpr bool DEBUG_ENABLED = false;
static constexpr uint32_t DEBUG_BAUD_RATE = 115200;
static constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 1000;

#endif

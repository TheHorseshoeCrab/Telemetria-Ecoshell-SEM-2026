#include "MPU6050.h"
#include "Setup.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

static bool sensorReady = false;
static bool calibrated = false;
static uint16_t calibrationCount = 0;
static uint32_t lastSampleUs = 0;
static uint32_t lastAttemptUs = 0;
static uint32_t lastDataUs = 0;      // Última respuesta útil del sensor
static uint32_t lastInitMs = 0;
static uint32_t maxSampleIntervalUs = 0;
static float gyroSum[3] = {};
static float accelSum[3] = {};
static float gyroBias[3] = {};
static float previousAcceleration[3] = {};
static float accelerationScale = 1.0f;
// Cuaternión que rota del marco del vehículo al marco inicial nivelado.
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;

static bool writeRegister(uint8_t address, uint8_t value)
{
    Wire.beginTransmission(MPU_I2C_ADDRESS);
    Wire.write(address);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool readRegisters(uint8_t address, uint8_t *data, uint8_t length)
{
    Wire.beginTransmission(MPU_I2C_ADDRESS);
    Wire.write(address);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(MPU_I2C_ADDRESS, static_cast<size_t>(length), true) != length) {
        return false;
    }
    for (uint8_t i = 0; i < length; ++i) {
        data[i] = static_cast<uint8_t>(Wire.read());
    }
    return true;
}

static int16_t readSigned16(const uint8_t *data)
{
    const uint16_t raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return static_cast<int16_t>((raw & 0x8000U) ? static_cast<int32_t>(raw) - 65536 : raw);
}

static void clearEstimator(Telemetria &telemetry)
{
    calibrated = false;
    calibrationCount = 0;
    telemetry.imuCalibrated = false;
    for (size_t i = 0; i < 3; ++i) {
        gyroSum[i] = accelSum[i] = gyroBias[i] = previousAcceleration[i] = 0.0f;
        telemetry.velocity_mps[i] = telemetry.position_m[i] = 0.0f;
        telemetry.acceleration_mps2[i] = telemetry.angularRate_dps[i] = 0.0f;
    }
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    accelerationScale = 1.0f;
}

static void invalidateIMU(Telemetria &telemetry)
{
    clearEstimator(telemetry);
    telemetry.mpuOk = false;
    lastSampleUs = 0;
}

uint32_t takeImuMaxIntervalUs()
{
    const uint32_t value = maxSampleIntervalUs;
    maxSampleIntervalUs = 0;
    return value;
}

bool beginMPU6050()
{
    static bool wireStarted = false;
    if (!wireStarted) {
        Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
        Wire.setClock(400000);
        Wire.setTimeOut(5);
        wireStarted = true;
    }

    // Esperas breves (~110 ms) solo al arrancar o al reintentar un sensor caído.
    lastInitMs = millis();
    lastDataUs = micros();
    sensorReady = false;
    uint8_t identity = 0;
    if (!readRegisters(0x75, &identity, 1) || (identity & 0x7E) != 0x68) {
        return false;
    }
    if (!writeRegister(0x6B, 0x80)) {
        return false;
    }
    delay(100);
    if (!writeRegister(0x6B, 0x01)) {
        return false;
    }
    delay(10);

    // DLPF 44 Hz, divisor 9 -> 100 Hz, gyro ±250 °/s, acelerómetro ±4 g.
    sensorReady = writeRegister(0x1A, 0x03) &&
                  writeRegister(0x19, static_cast<uint8_t>(IMU_SAMPLE_INTERVAL_US / 1000U - 1U)) &&
                  writeRegister(0x1B, 0x00) && writeRegister(0x1C, 0x08) &&
                  writeRegister(0x38, 0x01);
    calibrated = false;
    calibrationCount = 0;
    lastSampleUs = lastAttemptUs = 0;
    for (size_t i = 0; i < 3; ++i) {
        gyroSum[i] = accelSum[i] = 0.0f;
    }
    return sensorReady;
}

void updateMPU6050(Telemetria &telemetry)
{
    if (!sensorReady) {
        telemetry.mpuOk = telemetry.imuCalibrated = false;
        // Reintentar periódicamente: el sensor pudo reiniciarse por una caída de
        // tensión (queda dormido) o no estar listo al arrancar.
        if (millis() - lastInitMs >= IMU_REINIT_INTERVAL_MS) {
            clearEstimator(telemetry);
            beginMPU6050();
        }
        return;
    }
    const uint32_t attemptUs = micros();
    // Consultar como máximo cada 1 ms. Solo integrar muestras nuevas.
    if (attemptUs - lastAttemptUs < 1000) {
        return;
    }
    lastAttemptUs = attemptUs;

    // Un error I2C aislado o una muestra que tarda se reintentan en la siguiente
    // vuelta. Solo si el sensor no entrega datos durante más de 50 ms se declara
    // caído y se programa su reinicialización.
    const bool sensorSilent = attemptUs - lastDataUs > IMU_MAX_GAP_US;
    uint8_t interruptStatus = 0;
    if (!readRegisters(0x3A, &interruptStatus, 1) || (interruptStatus & 0x01) == 0) {
        if (sensorSilent) {
            invalidateIMU(telemetry);
            sensorReady = false;
        }
        return;
    }

    uint8_t raw[14];
    if (!readRegisters(0x3B, raw, sizeof(raw))) {
        if (sensorSilent) {
            invalidateIMU(telemetry);
            sensorReady = false;
        }
        return;
    }
    lastDataUs = attemptUs;
    float sensorAccel[3], sensorGyro[3], accel[3], gyro[3];
    for (size_t i = 0; i < 3; ++i) {
        const int16_t rawAccel = readSigned16(raw + 2 * i);
        const int16_t rawGyro = readSigned16(raw + 8 + 2 * i);
        // Muestra saturada (bache, golpe): descartarla sin borrar la estimación.
        // La siguiente muestra válida cubre el intervalo con su dt real; si la
        // saturación dura más de 50 ms, la regla de pausa larga recalibra.
        if (abs(static_cast<int>(rawAccel)) >= 32760 || abs(static_cast<int>(rawGyro)) >= 32760) {
            return;
        }
        sensorAccel[i] = rawAccel * (GRAVITY_MPS2 / 8192.0f);
        sensorGyro[i] = rawGyro / 131.0f;
    }
    for (size_t i = 0; i < 3; ++i) {
        accel[i] = IMU_AXIS_SIGN[i] * sensorAccel[IMU_AXIS_MAP[i]];
        gyro[i] = IMU_AXIS_SIGN[i] * sensorGyro[IMU_AXIS_MAP[i]];
    }

    const uint32_t sampleUs = micros();
    const uint32_t elapsedUs = lastSampleUs == 0 ? IMU_SAMPLE_INTERVAL_US : sampleUs - lastSampleUs;
    lastSampleUs = sampleUs;
    if (elapsedUs > maxSampleIntervalUs) {
        maxSampleIntervalUs = elapsedUs;
    }
    // Si se pierden más de 50 ms, recalibrar; no inventar el movimiento perdido.
    if (elapsedUs > IMU_MAX_GAP_US) {
        clearEstimator(telemetry);
    }
    telemetry.mpuOk = true;
    const float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);

    if (!calibrated) {
        // Comprobar reposo aproximado. El conductor debe permanecer inmóvil.
        // El gyro se compara contra su promedio, no contra cero, porque aún no
        // se conoce el bias (puede superar 3 °/s en un sensor sano).
        bool still = fabsf(norm - GRAVITY_MPS2) < 1.0f;
        for (size_t i = 0; i < 3; ++i) {
            still = still && fabsf(gyro[i]) < IMU_STILL_GYRO_MAX_DPS &&
                    (calibrationCount == 0 ||
                     fabsf(gyro[i] - gyroSum[i] / calibrationCount) < IMU_STILL_GYRO_DEVIATION_DPS);
        }
        if (!still) {
            clearEstimator(telemetry);
            // Conservar la lectura real del gyro (sin corregir bias). Antes quedaba
            // en cero con STATUS_MPU activo: un cero "válido" falso en el paquete.
            for (size_t i = 0; i < 3; ++i) {
                telemetry.angularRate_dps[i] = gyro[i];
            }
            return;
        }
        for (size_t i = 0; i < 3; ++i) {
            gyroSum[i] += gyro[i];
            accelSum[i] += accel[i];
            telemetry.angularRate_dps[i] = gyro[i];
        }
        if (++calibrationCount < IMU_CALIBRATION_SAMPLES) {
            return;
        }
        float mean[3];
        for (size_t i = 0; i < 3; ++i) {
            mean[i] = accelSum[i] / IMU_CALIBRATION_SAMPLES;
            gyroBias[i] = gyroSum[i] / IMU_CALIBRATION_SAMPLES;
        }
        const float meanNorm = sqrtf(mean[0]*mean[0] + mean[1]*mean[1] + mean[2]*mean[2]);
        accelerationScale = GRAVITY_MPS2 / meanNorm;
        const float roll = atan2f(mean[1], mean[2]);
        const float pitch = atan2f(-mean[0], sqrtf(mean[1]*mean[1] + mean[2]*mean[2]));
        // Inicializar inclinación y establecer yaw = 0, sin referencia al norte.
        q[0] = cosf(roll/2) * cosf(pitch/2);
        q[1] = sinf(roll/2) * cosf(pitch/2);
        q[2] = cosf(roll/2) * sinf(pitch/2);
        q[3] = -sinf(roll/2) * sinf(pitch/2);
        calibrated = telemetry.imuCalibrated = true;
        for (size_t i = 0; i < 3; ++i) {
            telemetry.angularRate_dps[i] = gyro[i] - gyroBias[i];
        }
        return;
    }

    const float dt = elapsedUs * 1.0e-6f;
    float omega[3];
    for (size_t i = 0; i < 3; ++i) {
        telemetry.angularRate_dps[i] = gyro[i] - gyroBias[i];
        omega[i] = telemetry.angularRate_dps[i] * DEG_TO_RAD_F;
    }

    // Filtro complementario en cuaterniones: corregir inclinación con gravedad.
    // Limitar la corrección cuando la magnitud indica aceleración dinámica.
    if (fabsf(norm * accelerationScale - GRAVITY_MPS2) < 0.8f) {
        const float gravityBody[3] = {
            2*(q[1]*q[3] - q[0]*q[2]),
            2*(q[2]*q[3] + q[0]*q[1]),
            1 - 2*(q[1]*q[1] + q[2]*q[2])
        };
        const float measured[3] = {accel[0]/norm, accel[1]/norm, accel[2]/norm};
        omega[0] += (measured[1]*gravityBody[2] - measured[2]*gravityBody[1]) / IMU_FILTER_TAU_S;
        omega[1] += (measured[2]*gravityBody[0] - measured[0]*gravityBody[2]) / IMU_FILTER_TAU_S;
        omega[2] += (measured[0]*gravityBody[1] - measured[1]*gravityBody[0]) / IMU_FILTER_TAU_S;
    }
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    q[0] += 0.5f * (-x*omega[0] - y*omega[1] - z*omega[2]) * dt;
    q[1] += 0.5f * ( w*omega[0] + y*omega[2] - z*omega[1]) * dt;
    q[2] += 0.5f * ( w*omega[1] - x*omega[2] + z*omega[0]) * dt;
    q[3] += 0.5f * ( w*omega[2] + x*omega[1] - y*omega[0]) * dt;
    const float qNorm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (!isfinite(qNorm) || qNorm < 0.1f) {
        invalidateIMU(telemetry);
        return;
    }
    for (float &component : q) {
        component /= qNorm;
    }

    // Rotar la fuerza específica al marco inicial y retirar la gravedad.
    const float ax = accel[0] * accelerationScale;
    const float ay = accel[1] * accelerationScale;
    const float az = accel[2] * accelerationScale;
    const float qw=q[0], qx=q[1], qy=q[2], qz=q[3];
    float linear[3] = {
        (1-2*(qy*qy+qz*qz))*ax + 2*(qx*qy-qw*qz)*ay + 2*(qx*qz+qw*qy)*az,
        2*(qx*qy+qw*qz)*ax + (1-2*(qx*qx+qz*qz))*ay + 2*(qy*qz-qw*qx)*az,
        2*(qx*qz-qw*qy)*ax + 2*(qy*qz+qw*qx)*ay + (1-2*(qx*qx+qy*qy))*az - GRAVITY_MPS2
    };
    // Integración trapezoidal. Sin referencia externa, velocidad y posición derivan.
    // No se fuerza velocidad cero solo por medir aceleración pequeña.
    for (size_t i = 0; i < 3; ++i) {
        const float oldVelocity = telemetry.velocity_mps[i];
        telemetry.velocity_mps[i] += 0.5f * (previousAcceleration[i] + linear[i]) * dt;
        telemetry.position_m[i] += 0.5f * (oldVelocity + telemetry.velocity_mps[i]) * dt;
        telemetry.acceleration_mps2[i] = linear[i];
        previousAcceleration[i] = linear[i];
        if (!isfinite(telemetry.position_m[i]) || !isfinite(telemetry.velocity_mps[i])) {
            invalidateIMU(telemetry);
            return;
        }
    }
    telemetry.imuCalibrated = true;
}

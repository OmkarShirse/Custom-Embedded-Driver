#pragma once

/**
 * @file mpu6050.h
 * @brief High-efficiency, register-level MPU-6050 6-axis IMU driver for ESP32.
 *
 * Implements register initialization, atomic 14-byte continuous burst reads
 * for all axes and temperature, sensitivity scaling, and data-ready interrupt setup.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "mpu6050_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default sensitivities */
#define MPU6050_ACCEL_SENS_2G   16384.0f    /* LSB/g    for ±2g   */
#define MPU6050_ACCEL_SENS_4G   8192.0f     /* LSB/g    for ±4g   */
#define MPU6050_ACCEL_SENS_8G   4096.0f     /* LSB/g    for ±8g   */
#define MPU6050_ACCEL_SENS_16G  2048.0f     /* LSB/g    for ±16g  */

#define MPU6050_GYRO_SENS_250   131.0f      /* LSB/(°/s) for ±250°/s  */
#define MPU6050_GYRO_SENS_500   65.5f       /* LSB/(°/s) for ±500°/s  */
#define MPU6050_GYRO_SENS_1000  32.8f       /* LSB/(°/s) for ±1000°/s */
#define MPU6050_GYRO_SENS_2000  16.4f       /* LSB/(°/s) for ±2000°/s */

/**
 * @brief Raw 16-bit sensor data structure directly from registers.
 */
typedef struct {
    int16_t accel_x;    /**< Raw accelerometer X-axis */
    int16_t accel_y;    /**< Raw accelerometer Y-axis */
    int16_t accel_z;    /**< Raw accelerometer Z-axis */
    int16_t temp_raw;   /**< Raw temperature reading */
    int16_t gyro_x;     /**< Raw gyroscope X-axis */
    int16_t gyro_y;     /**< Raw gyroscope Y-axis */
    int16_t gyro_z;     /**< Raw gyroscope Z-axis */
} mpu6050_raw_data_t;

/**
 * @brief Scaled engineering units representation.
 */
typedef struct {
    float ax_g;         /**< Accelerometer X in Earth gravity (g) */
    float ay_g;         /**< Accelerometer Y in Earth gravity (g) */
    float az_g;         /**< Accelerometer Z in Earth gravity (g) */
    float temp_c;       /**< Temperature in degrees Celsius (°C) */
    float gx_dps;       /**< Gyroscope X angular rate in degrees/sec (°/s) */
    float gy_dps;       /**< Gyroscope Y angular rate in degrees/sec (°/s) */
    float gz_dps;       /**< Gyroscope Z angular rate in degrees/sec (°/s) */
} mpu6050_scaled_data_t;

/**
 * @brief Initialization configuration parameters.
 */
typedef struct {
    i2c_port_t port;        /**< I2C master port (e.g. I2C_NUM_0) */
    uint8_t dev_addr;       /**< 7-bit device address (0x68 or 0x69) */
    uint8_t accel_fs;       /**< Accelerometer full-scale range (e.g. MPU6050_ACCEL_FS_2G) */
    uint8_t gyro_fs;        /**< Gyroscope full-scale range (e.g. MPU6050_GYRO_FS_250DPS) */
    uint8_t dlpf_cfg;       /**< Digital Low-Pass Filter mode (e.g. MPU6050_DLPF_44HZ) */
    uint8_t smplrt_div;     /**< Sample rate divider: SampleRate = GyroOutputRate / (1 + div) */
    bool enable_interrupt;  /**< Enable Data-Ready interrupt on INT pin */
} mpu6050_config_t;

/**
 * @brief Verify connection by reading and validating the WHO_AM_I register.
 *
 * @param port     I2C port
 * @param dev_addr 7-bit I2C device address
 * @return ESP_OK if device responded with 0x68, ESP_ERR_NOT_FOUND or I2C error otherwise
 */
esp_err_t mpu6050_test_connection(i2c_port_t port, uint8_t dev_addr);

/**
 * @brief Initialize MPU6050 sensor with custom settings.
 *
 * Performs:
 *   1. Connection verification (WHO_AM_I)
 *   2. Reset and wake up via PWR_MGMT_1 (clock source = PLL with X Gyro)
 *   3. Digital Low-Pass Filter (DLPF) configuration
 *   4. Sample rate divider configuration
 *   5. Accelerometer and Gyroscope full scale ranges
 *   6. Interrupt pin and Data-Ready enable (if configured)
 *
 * @param config Pointer to configuration struct
 * @return ESP_OK on success, or error code
 */
esp_err_t mpu6050_init(const mpu6050_config_t *config);

/**
 * @brief Quick setup with standard defaults (±2g, ±250°/s, 44Hz DLPF, INT enabled).
 *
 * @param port     I2C port
 * @param dev_addr 7-bit I2C device address
 * @return ESP_OK on success, or error code
 */
esp_err_t mpu6050_init_default(i2c_port_t port, uint8_t dev_addr);

/**
 * @brief Read all 6 axes and temperature in a single atomic 14-byte burst read.
 *
 * Reads registers 0x3B through 0x48 in one continuous I2C transfer:
 * [ACCEL_X_H, ACCEL_X_L, ACCEL_Y_H, ACCEL_Y_L, ACCEL_Z_H, ACCEL_Z_L,
 *  TEMP_H,    TEMP_L,    GYRO_X_H,  GYRO_X_L,  GYRO_Y_H,  GYRO_Y_L,  GYRO_Z_H,  GYRO_Z_L]
 *
 * @param port     I2C port
 * @param dev_addr 7-bit I2C device address
 * @param raw      Pointer to structure receiving 16-bit parsed signed integers
 * @return ESP_OK on success, or error code
 */
esp_err_t mpu6050_read_raw(i2c_port_t port, uint8_t dev_addr, mpu6050_raw_data_t *raw);

/**
 * @brief Convert raw integer sensor readings into engineering units (g, °/s, °C).
 *
 * @param raw        Pointer to input raw data
 * @param scaled     Pointer to output scaled data
 * @param accel_sens Accelerometer sensitivity (e.g. MPU6050_ACCEL_SENS_2G = 16384.0f)
 * @param gyro_sens  Gyroscope sensitivity (e.g. MPU6050_GYRO_SENS_250 = 131.0f)
 */
void mpu6050_scale_data(const mpu6050_raw_data_t *raw,
                        mpu6050_scaled_data_t *scaled,
                        float accel_sens,
                        float gyro_sens);

/**
 * @brief Configure INT pin for Data-Ready interrupt generation.
 *
 * Sets INT pin to Active High, Push-Pull, 50us pulse, and enables DATA_RDY_EN.
 *
 * @param port     I2C port
 * @param dev_addr 7-bit I2C device address
 * @return ESP_OK on success, or error code
 */
esp_err_t mpu6050_enable_data_ready_interrupt(i2c_port_t port, uint8_t dev_addr);

#ifdef __cplusplus
}
#endif

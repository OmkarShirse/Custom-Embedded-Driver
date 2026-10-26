#pragma once

/**
 * @file i2c_master.h
 * @brief Low-level I2C master driver for ESP32.
 *
 * Provides single-byte and multi-byte burst read/write transactions
 * over the ESP32 hardware I2C peripheral. Supports standard mode (100kHz)
 * and fast mode (400kHz).
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_DEFAULT_PORT     I2C_NUM_0
#define I2C_MASTER_DEFAULT_SDA      GPIO_NUM_21
#define I2C_MASTER_DEFAULT_SCL      GPIO_NUM_22
#define I2C_MASTER_DEFAULT_FREQ_HZ  400000      /* 400 kHz Fast-mode */
#define I2C_MASTER_TIMEOUT_MS       100

/**
 * @brief Initialize the I2C master peripheral with custom pin and speed settings.
 *
 * NOTE: For 400 kHz Fast-mode operation, external pull-up resistors (2.2kΩ - 4.7kΩ)
 * on SDA and SCL are mandatory. The ESP32 internal weak pull-ups (~45kΩ) do not meet
 * Fast-mode rise time specifications (tr <= 300ns).
 *
 * @param port         I2C port number (e.g. I2C_NUM_0)
 * @param sda_pin      GPIO number for SDA line
 * @param scl_pin      GPIO number for SCL line
 * @param clk_speed_hz I2C clock frequency in Hz (e.g. 100000 or 400000)
 * @return ESP_OK on success, or an error code from driver/i2c
 */
esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed_hz);

/**
 * @brief Convenience function to initialize I2C master on port 0 with default pins (21/22) at 400kHz.
 *
 * @return ESP_OK on success, or error code
 */
esp_err_t i2c_master_init_default(void);

/**
 * @brief Deinitialize the I2C master driver.
 *
 * @param port I2C port number
 * @return ESP_OK on success, or error code
 */
esp_err_t i2c_master_deinit(i2c_port_t port);

/**
 * @brief Write a single byte to a slave device register.
 *
 * Transaction sequence:
 *   START -> [DevAddr+W] -> ACK -> [RegAddr] -> ACK -> [Data] -> ACK -> STOP
 *
 * @param port      I2C port number
 * @param dev_addr  7-bit I2C slave address
 * @param reg_addr  Target register address on the slave
 * @param data      Byte to write
 * @return ESP_OK on success, ESP_ERR_TIMEOUT or ESP_FAIL on failure
 */
esp_err_t i2c_master_write_byte(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Read a single byte from a slave device register.
 *
 * Transaction sequence:
 *   START -> [DevAddr+W] -> ACK -> [RegAddr] -> ACK ->
 *   START (Repeated) -> [DevAddr+R] -> ACK -> [Data] -> NACK -> STOP
 *
 * @param port      I2C port number
 * @param dev_addr  7-bit I2C slave address
 * @param reg_addr  Target register address on the slave
 * @param data      Pointer to store received byte
 * @return ESP_OK on success, or error code
 */
esp_err_t i2c_master_read_byte(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Read multiple contiguous bytes starting from reg_addr in one single transaction.
 *
 * Transaction sequence:
 *   START -> [DevAddr+W] -> ACK -> [StartReg] -> ACK ->
 *   START (Repeated) -> [DevAddr+R] -> ACK ->
 *   [Byte 0] -> ACK -> [Byte 1] -> ACK ... -> [Byte N-1] -> NACK -> STOP
 *
 * @param port      I2C port number
 * @param dev_addr  7-bit I2C slave address
 * @param start_reg Starting register address
 * @param buf       Caller-allocated buffer to receive data
 * @param len       Number of bytes to read
 * @return ESP_OK on success, or error code
 */
esp_err_t i2c_master_read_burst(i2c_port_t port, uint8_t dev_addr, uint8_t start_reg, uint8_t *buf, size_t len);

/**
 * @brief Write multiple contiguous bytes starting from start_reg in one transaction.
 *
 * @param port      I2C port number
 * @param dev_addr  7-bit I2C slave address
 * @param start_reg Starting register address
 * @param buf       Data buffer to transmit
 * @param len       Number of bytes to write
 * @return ESP_OK on success, or error code
 */
esp_err_t i2c_master_write_burst(i2c_port_t port, uint8_t dev_addr, uint8_t start_reg, const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

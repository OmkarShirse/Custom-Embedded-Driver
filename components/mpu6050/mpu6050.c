#include "mpu6050.h"
#include "i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

esp_err_t mpu6050_test_connection(i2c_port_t port, uint8_t dev_addr)
{
    uint8_t who_am_i = 0;
    esp_err_t err = i2c_master_read_byte(port, dev_addr, MPU6050_REG_WHO_AM_I, &who_am_i);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I at 0x%02X: %s", dev_addr, esp_err_to_name(err));
        return err;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch! Expected 0x%02X, got 0x%02X",
                 MPU6050_WHO_AM_I_VAL, who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "MPU6050 detected at address 0x%02X (WHO_AM_I = 0x%02X)", dev_addr, who_am_i);
    return ESP_OK;
}

esp_err_t mpu6050_enable_data_ready_interrupt(i2c_port_t port, uint8_t dev_addr)
{
    /* INT pin config: Active High, Push-Pull, 50us pulse, clear on ANY read operation */
    esp_err_t err = i2c_master_write_byte(port, dev_addr, MPU6050_REG_INT_PIN_CFG, MPU6050_INT_CFG_INT_RD_CLEAR);
    if (err != ESP_OK) return err;

    /* Enable DATA_RDY_EN (bit 0) */
    err = i2c_master_write_byte(port, dev_addr, MPU6050_REG_INT_ENABLE, MPU6050_INT_EN_DATA_RDY);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Data-ready interrupt enabled on INT pin");
    return ESP_OK;
}

esp_err_t mpu6050_init(const mpu6050_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_port_t port = config->port;
    uint8_t addr    = config->dev_addr;

    /* 1. Verify WHO_AM_I */
    esp_err_t err = mpu6050_test_connection(port, addr);
    if (err != ESP_OK) return err;

    /* 2. Device reset to clear any stale state */
    err = i2c_master_write_byte(port, addr, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 3. Wake up and set clock source to PLL with X axis gyroscope reference (much more stable than internal 8MHz RC) */
    err = i2c_master_write_byte(port, addr, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_X);
    if (err != ESP_OK) return err;

    /* 4. Configure Sample Rate Divider */
    err = i2c_master_write_byte(port, addr, MPU6050_REG_SMPLRT_DIV, config->smplrt_div);
    if (err != ESP_OK) return err;

    /* 5. Configure Digital Low Pass Filter */
    err = i2c_master_write_byte(port, addr, MPU6050_REG_CONFIG, config->dlpf_cfg & 0x07);
    if (err != ESP_OK) return err;

    /* 6. Configure Gyroscope Full Scale Range */
    err = i2c_master_write_byte(port, addr, MPU6050_REG_GYRO_CONFIG, config->gyro_fs);
    if (err != ESP_OK) return err;

    /* 7. Configure Accelerometer Full Scale Range */
    err = i2c_master_write_byte(port, addr, MPU6050_REG_ACCEL_CONFIG, config->accel_fs);
    if (err != ESP_OK) return err;

    /* 8. Optional: Interrupt Enable */
    if (config->enable_interrupt) {
        err = mpu6050_enable_data_ready_interrupt(port, addr);
        if (err != ESP_OK) return err;
    }

    ESP_LOGI(TAG, "MPU6050 initialization successful");
    return ESP_OK;
}

esp_err_t mpu6050_init_default(i2c_port_t port, uint8_t dev_addr)
{
    mpu6050_config_t cfg = {
        .port = port,
        .dev_addr = dev_addr,
        .accel_fs = MPU6050_ACCEL_FS_2G,
        .gyro_fs = MPU6050_GYRO_FS_250DPS,
        .dlpf_cfg = MPU6050_DLPF_44HZ,
        .smplrt_div = 9,                     /* 1kHz / (1 + 9) = 100 Hz output rate */
        .enable_interrupt = true,
    };
    return mpu6050_init(&cfg);
}

esp_err_t mpu6050_read_raw(i2c_port_t port, uint8_t dev_addr, mpu6050_raw_data_t *raw)
{
    if (raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[14];
    esp_err_t err = i2c_master_read_burst(port, dev_addr, MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));
    if (err != ESP_OK) {
        return err;
    }

    /* High byte first, then low byte (big-endian from registers) */
    raw->accel_x  = (int16_t)(((uint16_t)buffer[0]  << 8) | buffer[1]);
    raw->accel_y  = (int16_t)(((uint16_t)buffer[2]  << 8) | buffer[3]);
    raw->accel_z  = (int16_t)(((uint16_t)buffer[4]  << 8) | buffer[5]);
    raw->temp_raw = (int16_t)(((uint16_t)buffer[6]  << 8) | buffer[7]);
    raw->gyro_x   = (int16_t)(((uint16_t)buffer[8]  << 8) | buffer[9]);
    raw->gyro_y   = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    raw->gyro_z   = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}

void mpu6050_scale_data(const mpu6050_raw_data_t *raw,
                        mpu6050_scaled_data_t *scaled,
                        float accel_sens,
                        float gyro_sens)
{
    if (raw == NULL || scaled == NULL || accel_sens <= 0.0f || gyro_sens <= 0.0f) {
        return;
    }

    scaled->ax_g   = (float)raw->accel_x / accel_sens;
    scaled->ay_g   = (float)raw->accel_y / accel_sens;
    scaled->az_g   = (float)raw->accel_z / accel_sens;

    /* Temperature formula from MPU6050 register map datasheet */
    scaled->temp_c = ((float)raw->temp_raw / 340.0f) + 36.53f;

    scaled->gx_dps = (float)raw->gyro_x / gyro_sens;
    scaled->gy_dps = (float)raw->gyro_y / gyro_sens;
    scaled->gz_dps = (float)raw->gyro_z / gyro_sens;
}

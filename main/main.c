#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/gpio.h"

#include "gpio_fast.h"
#include "i2c_master.h"
#include "mpu6050.h"
#include "filters.h"
#include "madgwick.h"

static const char *TAG = "app_main";

/* ==================== Pin Definitions ==================== */
#define PIN_LED_HEARTBEAT   GPIO_NUM_2      /* Standard ESP32 onboard LED */
#define PIN_LED_ACTIVITY    GPIO_NUM_5      /* Secondary status LED */
#define PIN_BUTTON          GPIO_NUM_19     /* User button (active low) */
#define PIN_MPU_INT         GPIO_NUM_4      /* MPU6050 INT data-ready pin */

#define PIN_I2C_SDA         GPIO_NUM_21     /* I2C Master SDA */
#define PIN_I2C_SCL         GPIO_NUM_22     /* I2C Master SCL */

/* ==================== FreeRTOS Pipeline Structs & Handles ==================== */
typedef struct {
    /* Complementary filter angles */
    float comp_roll_deg;
    float comp_pitch_deg;

    /* Madgwick AHRS quaternion & angles */
    float madg_roll_deg;
    float madg_pitch_deg;
    float madg_yaw_deg;
    float q0, q1, q2, q3;

    /* Filtered linear acceleration (g) */
    float ax_filtered_g;
    float ay_filtered_g;
    float az_filtered_g;

    /* Angular velocity (°/s) */
    float gx_dps;
    float gy_dps;
    float gz_dps;

    float temp_c;
    uint32_t sample_index;
} imu_telemetry_t;

static TaskHandle_t  s_sensor_task_handle     = NULL;
static QueueHandle_t s_raw_queue              = NULL;
static QueueHandle_t s_telemetry_queue        = NULL;

/* ==================== Hardware ISR ==================== */
static void IRAM_ATTR mpu6050_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_sensor_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_sensor_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ==================== FreeRTOS Tasks ==================== */

/**
 * @brief Task 1 (High Priority): Woken by hardware ISR, reads raw 14-byte sensor burst.
 */
static void sensor_task(void *pvParameters)
{
    mpu6050_raw_data_t raw_sample;
    ESP_LOGI("sensor_task", "Sensor acquisition task started (Priority 3)");

    while (1) {
        /* Wait for ISR notification (with 20ms fallback timeout if INT pin is unconnected) */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        esp_err_t err = mpu6050_read_raw(I2C_MASTER_DEFAULT_PORT,
                                         MPU6050_I2C_ADDR_DEFAULT,
                                         &raw_sample);
        if (err == ESP_OK) {
            /* Transfer raw sample to processing queue */
            if (xQueueSend(s_raw_queue, &raw_sample, 0) != pdPASS) {
                /* Queue full: drop oldest or log overrun */
            }
        }
    }
}

/**
 * @brief Task 2 (Medium Priority): Digital signal processing, unit conversion, and filter fusion.
 */
static void processing_task(void *pvParameters)
{
    mpu6050_raw_data_t raw;
    mpu6050_scaled_data_t scaled;
    imu_telemetry_t telem;

    /* Filter buffers */
    float ax_buf[MOVING_AVG_DEFAULT_WINDOW];
    float ay_buf[MOVING_AVG_DEFAULT_WINDOW];
    float az_buf[MOVING_AVG_DEFAULT_WINDOW];

    moving_avg_filter_t filter_ax, filter_ay, filter_az;
    moving_avg_init(&filter_ax, ax_buf, MOVING_AVG_DEFAULT_WINDOW);
    moving_avg_init(&filter_ay, ay_buf, MOVING_AVG_DEFAULT_WINDOW);
    moving_avg_init(&filter_az, az_buf, MOVING_AVG_DEFAULT_WINDOW);

    /* Complementary filter: 96% gyro, 4% accel */
    complementary_filter_t comp_filter;
    complementary_filter_init(&comp_filter, 0.96f);

    /* Madgwick quaternion AHRS filter: beta = 0.05 */
    madgwick_filter_t madg_filter;
    madgwick_init(&madg_filter, 0.05f);

    const float deg_to_rad = 3.14159265358979323846f / 180.0f;
    int64_t last_time_us = esp_timer_get_time();
    uint32_t count = 0;

    ESP_LOGI("processing_task", "Signal processing & DSP task started (Priority 2)");

    while (1) {
        if (xQueueReceive(s_raw_queue, &raw, portMAX_DELAY) == pdPASS) {
            int64_t now_us = esp_timer_get_time();
            float dt = (float)(now_us - last_time_us) / 1000000.0f;
            last_time_us = now_us;
            if (dt <= 0.0f || dt > 0.1f) {
                dt = 0.01f; /* Default to 100 Hz dt on first run or jitter spike */
            }

            /* Convert raw ADC values to physical units */
            mpu6050_scale_data(&raw, &scaled, MPU6050_ACCEL_SENS_2G, MPU6050_GYRO_SENS_250);

            /* Moving average filter on accelerometer axes */
            float ax_f = moving_avg_update(&filter_ax, scaled.ax_g);
            float ay_f = moving_avg_update(&filter_ay, scaled.ay_g);
            float az_f = moving_avg_update(&filter_az, scaled.az_g);

            /* 1. Complementary filter fusion for pitch and roll */
            complementary_filter_update(&comp_filter,
                                        ax_f, ay_f, az_f,
                                        scaled.gx_dps, scaled.gy_dps,
                                        dt);

            /* 2. Madgwick AHRS quaternion fusion */
            float gx_rad = scaled.gx_dps * deg_to_rad;
            float gy_rad = scaled.gy_dps * deg_to_rad;
            float gz_rad = scaled.gz_dps * deg_to_rad;
            madgwick_update_6dof(&madg_filter, gx_rad, gy_rad, gz_rad, ax_f, ay_f, az_f, dt);

            /* Package telemetry */
            telem.comp_roll_deg  = comp_filter.roll_deg;
            telem.comp_pitch_deg = comp_filter.pitch_deg;

            madgwick_get_euler_deg(&madg_filter,
                                   &telem.madg_roll_deg,
                                   &telem.madg_pitch_deg,
                                   &telem.madg_yaw_deg);
            madgwick_get_quaternion(&madg_filter,
                                    &telem.q0, &telem.q1, &telem.q2, &telem.q3);

            telem.ax_filtered_g  = ax_f;
            telem.ay_filtered_g  = ay_f;
            telem.az_filtered_g  = az_f;
            telem.gx_dps         = scaled.gx_dps;
            telem.gy_dps         = scaled.gy_dps;
            telem.gz_dps         = scaled.gz_dps;
            telem.temp_c         = scaled.temp_c;
            telem.sample_index   = ++count;

            xQueueSend(s_telemetry_queue, &telem, 0);
        }
    }
}

/**
 * @brief Task 3 (Low Priority): Telemetry formatting, status LED toggle, and serial monitor output.
 */
static void telemetry_task(void *pvParameters)
{
    imu_telemetry_t telem;
    ESP_LOGI("telemetry_task", "Telemetry reporter task started (Priority 1)");

    while (1) {
        if (xQueueReceive(s_telemetry_queue, &telem, portMAX_DELAY) == pdPASS) {
            /* Output formatted stream every 10 samples (approx 10Hz log rate at 100Hz sampling) */
            if ((telem.sample_index % 10) == 0) {
                ESP_LOGI("IMU", "[#%05" PRIu32 "] Comp: [R:%5.1f° P:%5.1f°] | Madgwick: [R:%5.1f° P:%5.1f° Y:%5.1f°] | Quat:[%.2f, %.2f, %.2f, %.2f]",
                         telem.sample_index,
                         telem.comp_roll_deg,
                         telem.comp_pitch_deg,
                         telem.madg_roll_deg,
                         telem.madg_pitch_deg,
                         telem.madg_yaw_deg,
                         telem.q0, telem.q1, telem.q2, telem.q3);

                /* Toggle heartbeat status LED using register-level fast GPIO */
                gpio_fast_toggle(PIN_LED_HEARTBEAT);
            }
        }
    }
}

/* ==================== Fast GPIO Benchmark Demo ==================== */
static void run_gpio_benchmark(void)
{
    ESP_LOGI(TAG, "--- Running Register-Level Fast GPIO Benchmark ---");
    const uint32_t N_ITER = 100000;

    int64_t t0 = esp_timer_get_time();
    for (uint32_t i = 0; i < N_ITER; i++) {
        gpio_fast_set(PIN_LED_ACTIVITY);
        gpio_fast_clear(PIN_LED_ACTIVITY);
    }
    int64_t t1 = esp_timer_get_time();
    int64_t fast_ns = ((t1 - t0) * 1000) / N_ITER;

    t0 = esp_timer_get_time();
    for (uint32_t i = 0; i < N_ITER; i++) {
        gpio_set_level(PIN_LED_ACTIVITY, 1);
        gpio_set_level(PIN_LED_ACTIVITY, 0);
    }
    t1 = esp_timer_get_time();
    int64_t idf_ns = ((t1 - t0) * 1000) / N_ITER;

    ESP_LOGI(TAG, "Fast GPIO set+clear: %" PRId64 " ns/pair", fast_ns);
    ESP_LOGI(TAG, "ESP-IDF   set+clear: %" PRId64 " ns/pair (%.1fx slower)",
             idf_ns, (float)idf_ns / (float)fast_ns);
    ESP_LOGI(TAG, "--------------------------------------------------");
}

/* ==================== Application Entrypoint ==================== */
void app_main(void)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "   ESP32 Custom Embedded Driver Pipeline Boot     ");
    ESP_LOGI(TAG, "==================================================");

    /* 1. Initialize Fast GPIOs */
    uint64_t led_mask = (1ULL << PIN_LED_HEARTBEAT) | (1ULL << PIN_LED_ACTIVITY);
    ESP_ERROR_CHECK(gpio_fast_config_mask(led_mask, GPIO_FAST_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_fast_config(PIN_BUTTON, GPIO_FAST_MODE_INPUT_PULLUP));

    /* Run quick benchmark demo */
    run_gpio_benchmark();

    /* 2. Initialize Hardware I2C Master Bus */
    ESP_ERROR_CHECK(i2c_master_init(I2C_MASTER_DEFAULT_PORT,
                                    PIN_I2C_SDA,
                                    PIN_I2C_SCL,
                                    I2C_MASTER_DEFAULT_FREQ_HZ));

    /* 3. Initialize MPU6050 IMU Sensor */
    mpu6050_config_t imu_cfg = {
        .port             = I2C_MASTER_DEFAULT_PORT,
        .dev_addr         = MPU6050_I2C_ADDR_DEFAULT,
        .accel_fs         = MPU6050_ACCEL_FS_2G,
        .gyro_fs          = MPU6050_GYRO_FS_250DPS,
        .dlpf_cfg         = MPU6050_DLPF_44HZ,
        .smplrt_div       = 9,                       /* 100 Hz output rate */
        .enable_interrupt = true,
    };

    esp_err_t err = mpu6050_init(&imu_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 sensor not found or error: %s (Pipeline running in demo fallback)",
                 esp_err_to_name(err));
    }

    /* 4. Configure GPIO Interrupt for MPU6050 INT pin */
    gpio_config_t int_conf = {
        .intr_type    = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << PIN_MPU_INT),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&int_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_MPU_INT, mpu6050_isr_handler, NULL);

    /* 5. Create Inter-Task Communication Queues */
    s_raw_queue       = xQueueCreate(10, sizeof(mpu6050_raw_data_t));
    s_telemetry_queue = xQueueCreate(10, sizeof(imu_telemetry_t));

    if (s_raw_queue == NULL || s_telemetry_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create FreeRTOS queues!");
        return;
    }

    /* 6. Launch FreeRTOS Task Pipeline */
    xTaskCreatePinnedToCore(sensor_task,     "sensor_task",     3072, NULL, 3, &s_sensor_task_handle, 1);
    xTaskCreatePinnedToCore(processing_task, "processing_task", 4096, NULL, 2, NULL,                  1);
    xTaskCreatePinnedToCore(telemetry_task,  "telemetry_task",  2560, NULL, 1, NULL,                  0);

    ESP_LOGI(TAG, "All FreeRTOS pipeline tasks pinned and running!");
}

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "gpio_fast.h"

static const char *TAG = "example";

#define LED_A   4
#define LED_B   5
#define LED_C   18
#define BUTTON  19
#define BENCH_PIN LED_A
#define N_ITER  100000u

#define LED_MASK ((1UL << LED_A) | (1UL << LED_B) | (1UL << LED_C))

static portMUX_TYPE bench_lock = portMUX_INITIALIZER_UNLOCKED;

static int64_t IRAM_ATTR bench_fast(void)
{
    portENTER_CRITICAL(&bench_lock);           /* no ISR jitter; ~10 ms, far below WDT */
    int64_t t0 = esp_timer_get_time();
    for (uint32_t i = 0; i < N_ITER; i++) { gpio_fast_set(BENCH_PIN); gpio_fast_clear(BENCH_PIN); }
    int64_t t1 = esp_timer_get_time();
    portEXIT_CRITICAL(&bench_lock);
    return t1 - t0;
}

static int64_t IRAM_ATTR bench_idf(void)
{
    portENTER_CRITICAL(&bench_lock);
    int64_t t0 = esp_timer_get_time();
    for (uint32_t i = 0; i < N_ITER; i++) { gpio_set_level(BENCH_PIN, 1); gpio_set_level(BENCH_PIN, 0); }
    int64_t t1 = esp_timer_get_time();
    portEXIT_CRITICAL(&bench_lock);
    return t1 - t0;
}

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_fast_config_mask(LED_MASK, GPIO_FAST_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_fast_config(BUTTON, GPIO_FAST_MODE_INPUT_PULLUP));

    gpio_fast_set_mask(LED_MASK);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_fast_clear_mask(LED_MASK);
    gpio_fast_write_mask((1UL << LED_A) | (1UL << LED_C), 1UL << LED_B);

    /* per-iteration cost = set+clear pair, loop overhead included */
    int64_t us_fast = bench_fast();
    int64_t us_idf  = bench_idf();
    ESP_LOGI(TAG, "fast: %" PRId64 " ns per set+clear", us_fast * 1000 / N_ITER);
    ESP_LOGI(TAG, "idf : %" PRId64 " ns per set+clear", us_idf  * 1000 / N_ITER);

    while (1) {
        if (!gpio_fast_read(BUTTON)) {         /* active-low */
            gpio_fast_toggle_mask_atomic(LED_MASK);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
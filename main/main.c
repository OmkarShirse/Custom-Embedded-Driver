#include "gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#define LED_A   GPIO_NUM_4
#define LED_B   GPIO_NUM_5
#define LED_C   GPIO_NUM_18
#define BUTTON  GPIO_NUM_19

#define LED_MASK ((1UL << LED_A) | (1UL << LED_B) | (1UL << LED_C))

static const char *TAG = "example";

void app_main(void)
{
    /* One-time setup — validated, not latency-critical */
    gpio_fast_config_mask(LED_MASK, GPIO_FAST_MODE_OUTPUT);
    gpio_fast_config(BUTTON, GPIO_FAST_MODE_INPUT_PULLUP);

    /* --- Single-pin hot path --- */
    gpio_fast_set(LED_A);      // atomic, no lock, inlined to one instruction sequence
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_fast_clear(LED_A);

    /* --- Batch hot path: all three LEDs toggled in ONE register write
     * instead of three separate calls — no visible skew between them. --- */
    gpio_fast_set_mask(LED_MASK);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_fast_clear_mask(LED_MASK);

    /* --- Mixed write: A and C on, B off, still just 2 register writes --- */
    gpio_fast_write_mask((1UL << LED_A) | (1UL << LED_C), (1UL << LED_B));

    /* --- Reading input --- */
    if (!gpio_fast_read(BUTTON)) { // pulled low = pressed
        ESP_LOGI(TAG, "button pressed");
    }

    /* --- Toggle: flip LED_A without knowing its current state --- */
    for (int i = 0; i < 6; i++) {
        gpio_fast_toggle(LED_A);   // blink 3 times
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    /* --- Batch toggle: all three LEDs flip state in two register writes --- */
    gpio_fast_set_mask(LED_MASK);          // all on
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_fast_toggle_mask(LED_MASK);       // all off (toggled from on)
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_fast_toggle_mask(LED_MASK);       // all on again

    /* --- Throughput demo: toggle LED_A as fast as possible and measure.
     * With gpio_set_level() this loop is dominated by critical-section
     * enter/exit; with gpio_fast_set/clear it's dominated by the actual
     * register write latency. Compare by swapping the calls below. */
    int64_t start = esp_timer_get_time();
    const int N = 100000;
    for (int i = 0; i < N; i++) {
        gpio_fast_set(LED_A);
        gpio_fast_clear(LED_A);
    }
    int64_t elapsed_us = esp_timer_get_time() - start;
    ESP_LOGI(TAG, "%d toggles in %lld us (%.1f ns/toggle)",
             N, elapsed_us, (elapsed_us * 1000.0) / N);

    /* --- Toggle throughput: compare gpio_fast_toggle vs set+clear --- */
    start = esp_timer_get_time();
    for (int i = 0; i < N; i++) {
        gpio_fast_toggle(LED_A);
    }
    elapsed_us = esp_timer_get_time() - start;
    ESP_LOGI(TAG, "%d gpio_fast_toggle calls in %lld us (%.1f ns/call)",
             N, elapsed_us, (elapsed_us * 1000.0) / N);
}

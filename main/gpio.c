#include "gpio.h"
#include "soc/io_mux_reg.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"

static const char *TAG = "gpio_fast";

/* GPIOs wired internally to the SPI flash on most ESP32 modules
 * (including the DevKit V1's WROOM-32). Touching these can hang or brick
 * the boot. We refuse to configure them regardless of requested mode. */
static inline bool is_flash_pin(uint8_t gpio_num)
{
    return gpio_num >= 6 && gpio_num <= 11;
}

/* Pins 34-39 have no output driver and no internal pull resistors —
 * input-only by hardware design, not by software choice. */
static inline bool is_input_only_pin(uint8_t gpio_num)
{
    return gpio_num >= 34 && gpio_num <= 39;
}

esp_err_t gpio_fast_config(uint8_t gpio_num, gpio_fast_mode_t mode)
{
    if (gpio_num > 39) {
        ESP_LOGE(TAG, "GPIO%d does not exist on ESP32", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    if (is_flash_pin(gpio_num)) {
        ESP_LOGE(TAG, "GPIO%d is reserved for SPI flash, refusing to touch it", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    if (is_input_only_pin(gpio_num) &&
        (mode == GPIO_FAST_MODE_OUTPUT)) {
        ESP_LOGE(TAG, "GPIO%d is input-only (34-39 have no output driver)", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    if (is_input_only_pin(gpio_num) &&
        (mode == GPIO_FAST_MODE_INPUT_PULLUP || mode == GPIO_FAST_MODE_INPUT_PULLDOWN)) {
        ESP_LOGE(TAG, "GPIO%d has no internal pull resistors (34-39)", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    /* Route the pin through the GPIO matrix as a plain GPIO signal and set
     * its drive/pull configuration via the ROM helper — this part is
     * identical in spirit to what gpio_config() does, we just skip its
     * extra bookkeeping (interrupt config, per-call struct parsing) that
     * this driver doesn't use. */
    esp_rom_gpio_pad_select_gpio(gpio_num);

    if (mode == GPIO_FAST_MODE_OUTPUT) {
        if (gpio_num < 32) {
            REG_WRITE(GPIO_ENABLE_W1TS_REG, 1UL << gpio_num);
        } else {
            REG_WRITE(GPIO_ENABLE1_W1TS_REG, 1UL << (gpio_num - 32));
        }
    } else {
        if (gpio_num < 32) {
            REG_WRITE(GPIO_ENABLE_W1TC_REG, 1UL << gpio_num);
        } else {
            REG_WRITE(GPIO_ENABLE1_W1TC_REG, 1UL << (gpio_num - 32));
        }
        esp_rom_gpio_pad_pulldown(gpio_num, mode == GPIO_FAST_MODE_INPUT_PULLDOWN);
        esp_rom_gpio_pad_pullup(gpio_num, mode == GPIO_FAST_MODE_INPUT_PULLUP);
    }

    return ESP_OK;
}

esp_err_t gpio_fast_config_mask(uint64_t pin_mask, gpio_fast_mode_t mode)
{
    for (uint8_t pin = 0; pin < 40; pin++) {
        if (pin_mask & (1ULL << pin)) {
            esp_err_t err = gpio_fast_config(pin, mode);
            if (err != ESP_OK) {
                return err; /* fail fast — partial config left as-is for debugging */
            }
        }
    }
    return ESP_OK;
}

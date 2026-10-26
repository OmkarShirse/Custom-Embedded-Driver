#include "gpio_fast.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "driver/rtc_io.h"

static const char *TAG = "gpio_fast";

portMUX_TYPE gpio_fast_lock = portMUX_INITIALIZER_UNLOCKED;

static bool is_pull(gpio_fast_mode_t m)
{
    return m == GPIO_FAST_MODE_INPUT_PULLUP || m == GPIO_FAST_MODE_INPUT_PULLDOWN;
}

static esp_err_t validate(uint8_t n, gpio_fast_mode_t mode)
{
    if (mode > GPIO_FAST_MODE_OUTPUT || n > 39) {
        ESP_LOGE(TAG, "GPIO%u: bad pin/mode", n);
        return ESP_ERR_INVALID_ARG;
    }
    bool valid = (n < 32) ? ((GPIO_FAST_VALID_MASK0 >> n) & 1u)
                          : ((GPIO_FAST_VALID_MASK1 >> (n - 32)) & 1u);
    if (!valid) {
        ESP_LOGE(TAG, "GPIO%u does not exist on ESP32", n);
        return ESP_ERR_INVALID_ARG;
    }
    if (n < 32 && ((GPIO_FAST_FLASH_MASK0 >> n) & 1u)) {
        ESP_LOGE(TAG, "GPIO%u is wired to SPI flash", n);
        return ESP_ERR_INVALID_ARG;
    }
    if (n >= 34 && (mode == GPIO_FAST_MODE_OUTPUT || is_pull(mode))) {
        ESP_LOGE(TAG, "GPIO%u is input-only, no output/pull", n);
        return ESP_ERR_INVALID_ARG;
    }
    if (n < 32 && ((GPIO_FAST_WARN_MASK0 >> n) & 1u)) {
        ESP_LOGW(TAG, "GPIO%u is strapping/console/PSRAM on some modules", n);
    }
    return ESP_OK;
}

static void apply(uint8_t n, gpio_fast_mode_t mode)
{
    if (rtc_gpio_is_valid_gpio((gpio_num_t)n)) {
        rtc_gpio_deinit((gpio_num_t)n);        /* release RTC mux on 0,2,4,12-15,25-27,32-39 */
    }
    esp_rom_gpio_pad_select_gpio(n);           /* IO_MUX func = GPIO */

    uint32_t mux = GPIO_PIN_MUX_REG[n];
    PIN_INPUT_ENABLE(mux);                     /* gpio_fast_read must work in OUTPUT too */
    PIN_PULLUP_DIS(mux);
    PIN_PULLDWN_DIS(mux);
    if (mode == GPIO_FAST_MODE_INPUT_PULLUP)   PIN_PULLUP_EN(mux);
    if (mode == GPIO_FAST_MODE_INPUT_PULLDOWN) PIN_PULLDWN_EN(mux);

    if (mode == GPIO_FAST_MODE_OUTPUT) {
        /* detach any peripheral signal, route simple GPIO_OUT to pad */
        esp_rom_gpio_connect_out_signal(n, SIG_GPIO_OUT_IDX, false, false);
        if (n < 32) REG_WRITE(GPIO_ENABLE_W1TS_REG,  (uint32_t)1 << (n & 0x1Fu));
        else        REG_WRITE(GPIO_ENABLE1_W1TS_REG, (uint32_t)1 << ((n - 32) & 0x1Fu));
    } else {
        if (n < 32) REG_WRITE(GPIO_ENABLE_W1TC_REG,  (uint32_t)1 << (n & 0x1Fu));
        else        REG_WRITE(GPIO_ENABLE1_W1TC_REG, (uint32_t)1 << ((n - 32) & 0x1Fu));
    }
}

esp_err_t gpio_fast_config(uint8_t gpio_num, gpio_fast_mode_t mode)
{
    esp_err_t err = validate(gpio_num, mode);
    if (err != ESP_OK) return err;
    apply(gpio_num, mode);
    return ESP_OK;
}

esp_err_t gpio_fast_config_mask(uint64_t pin_mask, gpio_fast_mode_t mode)
{
    if (pin_mask == 0) return ESP_ERR_INVALID_ARG;
    for (uint8_t n = 0; n < 64; n++) {         /* pass 1: validate all */
        if ((pin_mask >> n) & 1ULL) {
            esp_err_t err = validate(n, mode);
            if (err != ESP_OK) return err;
        }
    }
    for (uint8_t n = 0; n < 40; n++) {         /* pass 2: apply */
        if ((pin_mask >> n) & 1ULL) apply(n, mode);
    }
    return ESP_OK;
}
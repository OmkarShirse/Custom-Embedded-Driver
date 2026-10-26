#pragma once
/*
 * ESP32 register-level fast GPIO. Target: esp32 only (not S2/S3/C3: different regs).
 *
 * Rules:
 *  - Hot-path fns are always_inline: they run from the CALLER's section.
 *    For flash-cache-off safety, callers must be IRAM_ATTR.
 *  - set/clear/write(pin)/set_mask/clear_mask: single W1TS/W1TC write -> atomic.
 *  - toggle*: read-modify-write (ESP32 has no XOR reg). NOT atomic. Pin must have
 *    one owner, or use *_atomic and make every writer of that pin use it.
 *  - write_mask / toggle_mask = 2 APB writes: a brief mixed state is visible
 *    on the pins between them. Clear is issued first, then set.
 */
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_FAST_INLINE static inline __attribute__((always_inline))

/* ESP32 pin map */
#define GPIO_FAST_VALID_MASK0   0x0EEFFFFFUL /* 0-19,21-23,25-27 (no 20,24,28-31) */
#define GPIO_FAST_FLASH_MASK0   0x00000FC0UL /* 6-11 SPI flash: refused */
#define GPIO_FAST_VALID_MASK1   0x000000FFUL /* 32-39 -> bits 0-7 */
#define GPIO_FAST_INONLY_MASK1  0x000000FCUL /* 34-39 input-only, no pulls */
#define GPIO_FAST_WARN_MASK0    0x0003902FUL /* 0,1,2,3,5 strap/console, 12,15 strap, 16,17 PSRAM */

typedef enum {
    GPIO_FAST_MODE_INPUT = 0,
    GPIO_FAST_MODE_INPUT_PULLUP,
    GPIO_FAST_MODE_INPUT_PULLDOWN,
    GPIO_FAST_MODE_OUTPUT,
} gpio_fast_mode_t;

/* ---- setup (not hot path) ---- */
esp_err_t gpio_fast_config(uint8_t gpio_num, gpio_fast_mode_t mode);
/* Validates ALL pins first; applies none on error. Use 1ULL<<n for pins >=32. */
esp_err_t gpio_fast_config_mask(uint64_t pin_mask, gpio_fast_mode_t mode);

extern portMUX_TYPE gpio_fast_lock;

/* ---- single pin ---- */
GPIO_FAST_INLINE void gpio_fast_set(uint8_t n)
{
    if (n < 32) REG_WRITE(GPIO_OUT_W1TS_REG,  (uint32_t)1 << (n & 0x1Fu));
    else        REG_WRITE(GPIO_OUT1_W1TS_REG, (uint32_t)1 << ((n - 32) & 0x1Fu));
}

GPIO_FAST_INLINE void gpio_fast_clear(uint8_t n)
{
    if (n < 32) REG_WRITE(GPIO_OUT_W1TC_REG,  (uint32_t)1 << (n & 0x1Fu));
    else        REG_WRITE(GPIO_OUT1_W1TC_REG, (uint32_t)1 << ((n - 32) & 0x1Fu));
}

GPIO_FAST_INLINE void gpio_fast_write(uint8_t n, bool lvl)
{
    if (lvl) gpio_fast_set(n); else gpio_fast_clear(n);
}

GPIO_FAST_INLINE bool gpio_fast_read(uint8_t n)
{
    return (n < 32) ? ((REG_READ(GPIO_IN_REG)  >> (n & 0x1Fu))        & 1u)
                    : ((REG_READ(GPIO_IN1_REG) >> ((n - 32) & 0x1Fu)) & 1u);
}

/* Reads the output latch (GPIO_OUT), not the pad. */
GPIO_FAST_INLINE bool gpio_fast_get_out(uint8_t n)
{
    return (n < 32) ? ((REG_READ(GPIO_OUT_REG)  >> (n & 0x1Fu))        & 1u)
                    : ((REG_READ(GPIO_OUT1_REG) >> ((n - 32) & 0x1Fu)) & 1u);
}

/* NOT atomic (RMW). Single-owner pins only. */
GPIO_FAST_INLINE void gpio_fast_toggle(uint8_t n)
{
    if (gpio_fast_get_out(n)) gpio_fast_clear(n); else gpio_fast_set(n);
}

GPIO_FAST_INLINE void gpio_fast_toggle_atomic(uint8_t n)
{
    portENTER_CRITICAL_SAFE(&gpio_fast_lock);
    gpio_fast_toggle(n);
    portEXIT_CRITICAL_SAFE(&gpio_fast_lock);
}

/* ---- bank 0: GPIO0-31, mask bit n = GPIOn ---- */
GPIO_FAST_INLINE void gpio_fast_set_mask(uint32_t m)   { REG_WRITE(GPIO_OUT_W1TS_REG, m); }
GPIO_FAST_INLINE void gpio_fast_clear_mask(uint32_t m) { REG_WRITE(GPIO_OUT_W1TC_REG, m); }

GPIO_FAST_INLINE void gpio_fast_write_mask(uint32_t set_m, uint32_t clr_m)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, clr_m);
    REG_WRITE(GPIO_OUT_W1TS_REG, set_m);
}

GPIO_FAST_INLINE void gpio_fast_toggle_mask(uint32_t m) /* RMW, not atomic */
{
    uint32_t cur = REG_READ(GPIO_OUT_REG);
    REG_WRITE(GPIO_OUT_W1TC_REG, cur & m);
    REG_WRITE(GPIO_OUT_W1TS_REG, ~cur & m);
}

GPIO_FAST_INLINE void gpio_fast_toggle_mask_atomic(uint32_t m)
{
    portENTER_CRITICAL_SAFE(&gpio_fast_lock);
    gpio_fast_toggle_mask(m);
    portEXIT_CRITICAL_SAFE(&gpio_fast_lock);
}

GPIO_FAST_INLINE uint32_t gpio_fast_read_bank0(void) { return REG_READ(GPIO_IN_REG); }

/* ---- bank 1: GPIO32-39, mask bit 0 = GPIO32 ... bit 7 = GPIO39 ----
 * Only GPIO32/33 can drive outputs; 34-39 are input-only. */
GPIO_FAST_INLINE void gpio_fast_set_mask1(uint32_t m)   { REG_WRITE(GPIO_OUT1_W1TS_REG, m & 0xFFu); }
GPIO_FAST_INLINE void gpio_fast_clear_mask1(uint32_t m) { REG_WRITE(GPIO_OUT1_W1TC_REG, m & 0xFFu); }

GPIO_FAST_INLINE void gpio_fast_write_mask1(uint32_t set_m, uint32_t clr_m)
{
    REG_WRITE(GPIO_OUT1_W1TC_REG, clr_m & 0xFFu);
    REG_WRITE(GPIO_OUT1_W1TS_REG, set_m & 0xFFu);
}

GPIO_FAST_INLINE void gpio_fast_toggle_mask1(uint32_t m) /* RMW, not atomic */
{
    m &= 0xFFu;
    uint32_t cur = REG_READ(GPIO_OUT1_REG);
    REG_WRITE(GPIO_OUT1_W1TC_REG, cur & m);
    REG_WRITE(GPIO_OUT1_W1TS_REG, ~cur & m);
}

GPIO_FAST_INLINE uint32_t gpio_fast_read_bank1(void) { return REG_READ(GPIO_IN1_REG) & 0xFFu; }

#ifdef __cplusplus
}
#endif
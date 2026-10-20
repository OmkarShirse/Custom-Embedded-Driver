#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"     // IRAM_ATTR
#include "soc/soc.h"      // REG_WRITE / REG_READ
#include "soc/gpio_reg.h" // GPIO_OUT_REG etc.
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPIO_FAST_MODE_INPUT = 0,
    GPIO_FAST_MODE_OUTPUT,
    GPIO_FAST_MODE_INPUT_PULLUP,
    GPIO_FAST_MODE_INPUT_PULLDOWN,
} gpio_fast_mode_t;

/* ---- One-time setup (not latency-critical, full validation happens here) ---- */

/**
 * Configure a single pin's direction, pull resistors, and route it through
 * the GPIO matrix as a simple GPIO (not a peripheral signal).
 * Call once at boot for every pin you intend to drive with the fast API.
 *
 * Returns ESP_ERR_INVALID_ARG for flash pins (6-11) or output mode on an
 * input-only pin (34-39).
 */
esp_err_t gpio_fast_config(uint8_t gpio_num, gpio_fast_mode_t mode);

/**
 * Same as gpio_fast_config but for a batch of pins sharing the same mode.
 * Use this at startup instead of N calls to gpio_fast_config to configure
 * a whole bus/port in one go.
 */
esp_err_t gpio_fast_config_mask(uint64_t pin_mask, gpio_fast_mode_t mode);

/* ---- Hot path: single-pin ops (inlined, IRAM, no locks) ---- */

/** Drive `gpio_num` high. Atomic single register write. */
static inline void IRAM_ATTR gpio_fast_set(uint8_t gpio_num)
{
    if (gpio_num < 32) {
        REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << gpio_num);
    } else {
        REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (gpio_num - 32));
    }
}

/** Drive `gpio_num` low. Atomic single register write. */
static inline void IRAM_ATTR gpio_fast_clear(uint8_t gpio_num)
{
    if (gpio_num < 32) {
        REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << gpio_num);
    } else {
        REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (gpio_num - 32));
    }
}

/** Set `gpio_num` to `level` (0 or nonzero). Branch is on a compile-time-
 *  knowable literal in most call sites, so it usually optimizes to a single
 *  register write anyway. */
static inline void IRAM_ATTR gpio_fast_write(uint8_t gpio_num, bool level)
{
    if (level) {
        gpio_fast_set(gpio_num);
    } else {
        gpio_fast_clear(gpio_num);
    }
}

/** Read the instantaneous input level of `gpio_num` (works for both input
 *  and output pins — output pins read back their driven state). */
static inline bool IRAM_ATTR gpio_fast_read(uint8_t gpio_num)
{
    if (gpio_num < 32) {
        return (REG_READ(GPIO_IN_REG) >> gpio_num) & 0x1;
    } else {
        return (REG_READ(GPIO_IN1_REG) >> (gpio_num - 32)) & 0x1;
    }
}

/* ---- Hot path: batch ops on pins 0-31 (bank 0) ----
 * These are the real win: N pins toggled for the price of 1 register write
 * instead of N. Use a precomputed mask, e.g. (1UL<<4)|(1UL<<5)|(1UL<<18).
 */

/** Set every pin present in `mask` (bank 0, pins 0-31) high, atomically and
 *  simultaneously — no glitch window between pins like a software loop
 *  would have. */
static inline void IRAM_ATTR gpio_fast_set_mask(uint32_t mask)
{
    REG_WRITE(GPIO_OUT_W1TS_REG, mask);
}

/** Clear every pin present in `mask` (bank 0, pins 0-31), atomically. */
static inline void IRAM_ATTR gpio_fast_clear_mask(uint32_t mask)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, mask);
}

/**
 * Write an arbitrary combination of pins high/low in bank 0 in exactly two
 * register writes total, regardless of how many pins are touched:
 *   set_mask  = pins that should go high
 *   clear_mask = pins that should go low
 * (set_mask & clear_mask should be 0; if a bit is in both, set wins because
 * it's applied second here — reorder the two REG_WRITE lines if you want
 * clear to win instead.)
 */
static inline void IRAM_ATTR gpio_fast_write_mask(uint32_t set_mask, uint32_t clear_mask)
{
    REG_WRITE(GPIO_OUT_W1TC_REG, clear_mask);
    REG_WRITE(GPIO_OUT_W1TS_REG, set_mask);
}

/** Read all 32 pins of bank 0 (pins 0-31) in one register read. Mask off
 *  the bits you care about: (gpio_fast_read_bank0() >> 4) & 1 for pin 4. */
static inline uint32_t IRAM_ATTR gpio_fast_read_bank0(void)
{
    return REG_READ(GPIO_IN_REG);
}

/** Same as above for bank 1 (pins 32-39), returned right-aligned in bits
 *  0-7 of the result (bit 0 == GPIO32). */
static inline uint32_t IRAM_ATTR gpio_fast_read_bank1(void)
{
    return REG_READ(GPIO_IN1_REG) & 0xFF;
}

/** Toggle `gpio_num` output state. Reads current driven output register,
 *  then sets or clears accordingly — two register accesses but no CPU
 *  critical section required. Works for bank 0 and bank 1 pins. */
static inline void IRAM_ATTR gpio_fast_toggle(uint8_t gpio_num)
{
    if (gpio_num < 32) {
        uint32_t out = REG_READ(GPIO_OUT_REG);
        if (out & (1UL << gpio_num)) {
            REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << gpio_num);
        } else {
            REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << gpio_num);
        }
    } else {
        uint32_t out1 = REG_READ(GPIO_OUT1_REG);
        if (out1 & (1UL << (gpio_num - 32))) {
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (gpio_num - 32));
        } else {
            REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (gpio_num - 32));
        }
    }
}

/** Toggle every pin present in `mask` (bank 0, pins 0-31) in exactly two
 *  register writes — pins currently HIGH go LOW and vice versa, atomically
 *  within each bank. */
static inline void IRAM_ATTR gpio_fast_toggle_mask(uint32_t mask)
{
    uint32_t out = REG_READ(GPIO_OUT_REG);
    uint32_t set = mask & ~out;  /* bits that are 0 → must go high */
    uint32_t clr = mask &  out;  /* bits that are 1 → must go low  */
    REG_WRITE(GPIO_OUT_W1TC_REG, clr);
    REG_WRITE(GPIO_OUT_W1TS_REG, set);
}

/* ---- Hot path: batch ops on pins 32-39 (bank 1) ----
 * Mirror of the bank-0 mask API but for the upper 8 GPIO pins.
 * Mask bits are right-aligned: bit 0 == GPIO32, bit 7 == GPIO39.
 * Only bits 0-7 are significant; upper bits are ignored.
 */

/** Set every pin in `mask` (bank 1, pins 32-39) high, atomically. */
static inline void IRAM_ATTR gpio_fast_set_mask1(uint32_t mask)
{
    REG_WRITE(GPIO_OUT1_W1TS_REG, mask & 0xFF);
}

/** Clear every pin in `mask` (bank 1, pins 32-39), atomically. */
static inline void IRAM_ATTR gpio_fast_clear_mask1(uint32_t mask)
{
    REG_WRITE(GPIO_OUT1_W1TC_REG, mask & 0xFF);
}

/** Write bank 1 pins high/low in two register writes, same semantics as
 *  gpio_fast_write_mask() but for pins 32-39. */
static inline void IRAM_ATTR gpio_fast_write_mask1(uint32_t set_mask, uint32_t clear_mask)
{
    REG_WRITE(GPIO_OUT1_W1TC_REG, clear_mask & 0xFF);
    REG_WRITE(GPIO_OUT1_W1TS_REG, set_mask   & 0xFF);
}

/** Toggle every pin in `mask` (bank 1, pins 32-39). */
static inline void IRAM_ATTR gpio_fast_toggle_mask1(uint32_t mask)
{
    uint32_t out1 = REG_READ(GPIO_OUT1_REG) & 0xFF;
    uint32_t m    = mask & 0xFF;
    uint32_t set  = m & ~out1;
    uint32_t clr  = m &  out1;
    REG_WRITE(GPIO_OUT1_W1TC_REG, clr);
    REG_WRITE(GPIO_OUT1_W1TS_REG, set);
}

#ifdef __cplusplus
}
#endif

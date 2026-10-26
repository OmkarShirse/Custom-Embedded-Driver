/**
 * @file esp32_bare_gpio.c
 * @brief Bare-metal ESP32 GPIO blink — direct register access without ESP-IDF gpio driver
 *
 * Demonstrates manual W1TS/W1TC register manipulation on GPIO2 (built-in LED).
 * This is the low-level foundation that gpio_fast.h builds on, useful as a
 * reference when porting to bare-metal environments or understanding the
 * hardware before using the higher-level fast-path API.
 *
 * NOTE: In production use gpio_fast.h instead — it adds IRAM_ATTR, validation,
 *       and a cleaner API over the same underlying registers.
 */
#include <stdint.h>

// ESP32 GPIO register base addresses
#define DR_REG_GPIO_BASE 0x3FF44000

#define GPIO_ENABLE_REG          (DR_REG_GPIO_BASE + 0x20)
#define GPIO_ENABLE_W1TS_REG     (DR_REG_GPIO_BASE + 0x24)
#define GPIO_ENABLE_W1TC_REG     (DR_REG_GPIO_BASE + 0x28)

#define GPIO_OUT_REG             (DR_REG_GPIO_BASE + 0x04)
#define GPIO_OUT_W1TS_REG        (DR_REG_GPIO_BASE + 0x08)
#define GPIO_OUT_W1TC_REG        (DR_REG_GPIO_BASE + 0x0C)

// Simple macro to access registers
#define REG_WRITE(addr, val) (*((volatile uint32_t *)(addr)) = (val))
#define REG_READ(addr)       (*((volatile uint32_t *)(addr)))

// Delay loop (very rough)
void delay(volatile uint32_t count) {
    while(count--) {
        __asm__ volatile ("nop");
    }
}

void app_main(void)
{
    // Enable GPIO2 as output
    REG_WRITE(GPIO_ENABLE_W1TS_REG, (1 << 2));

    while (1) {
        // Set GPIO2 HIGH (LED ON)
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << 2));
        delay(500000);

        // Set GPIO2 LOW (LED OFF)
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << 2));
        delay(500000);
    }
}
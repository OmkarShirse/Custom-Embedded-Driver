# Examples

Standalone C examples that demonstrate register-level GPIO/peripheral programming concepts, complementary to the main `gpio_fast.h` fast-path driver.

---

## `esp32_bare_gpio.c`

**Platform:** ESP32 (any bare-metal / Arduino / ESP-IDF setup)

Shows direct `W1TS` / `W1TC` register manipulation on GPIO2 without going through any driver API. This is exactly what `gpio.h` compiles down to — useful for understanding the hardware or porting to environments without ESP-IDF headers.

```c
// Enable GPIO2 output
REG_WRITE(GPIO_ENABLE_W1TS_REG, (1 << 2));

// Blink: set HIGH → delay → set LOW → delay
REG_WRITE(GPIO_OUT_W1TS_REG, (1 << 2));
delay(500000);
REG_WRITE(GPIO_OUT_W1TC_REG, (1 << 2));
```

---

## `stm8/dht11_led.c`

**Platform:** STM8S103F3 (compiled with [SDCC](http://sdcc.sourceforge.net/))

Bare-metal driver for the DHT11 temperature/humidity sensor on an STM8 microcontroller, using direct register access (no STM8 standard peripheral library). Drives an LED alert based on threshold conditions.

**Demonstrates:**
- STM8 Port D direction register (`DDR`) / output register (`ODR`) / input register (`IDR`) manipulation
- Bit-bang 1-Wire-style DHT11 protocol (pull-low start, bit timing, checksum)
- `__asm__("nop")` busy-delay calibrated for 16 MHz clock
- Alert logic: fast blink on bad environment, heartbeat blink on good reading

**Build:**
```bash
sdcc -mstm8 dht11_led.c -o dht11_led.ihx
# Flash with stm8flash or ST Visual Programmer
```

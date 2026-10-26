/**
 * @file dht11_led.c
 * @brief STM8 bare-metal DHT11 temperature/humidity driver with LED alert
 *
 * Target MCU : STM8S103F3 (SDCC toolchain)
 * Register map: STM8 Port D — PD2 = DHT11 data, PD4 = LED
 *
 * Behaviour:
 *  - Reads DHT11 every 2 seconds
 *  - Fast-blink LED (6×) if temperature > 30°C or humidity < 30%
 *  - Single quick blink on good reading
 *  - Two quick blinks on sensor read error
 */
#include <stdint.h>

#define CLK_CKDIVR (*(volatile uint8_t*)0x50C6)
#define PD_ODR (*(volatile uint8_t*)0x500F)
#define PD_IDR (*(volatile uint8_t*)0x5010)
#define PD_DDR (*(volatile uint8_t*)0x5011)
#define PD_CR1 (*(volatile uint8_t*)0x5012)

#define DHT_PIN 2
#define LED_PIN 4

void delay_us(uint16_t us) {
    while (us--) { __asm__("nop"); __asm__("nop"); __asm__("nop"); }
}
void delay_ms(uint16_t ms) { while (ms--) delay_us(1000); }

void dht_set_output(void) { PD_DDR |= (1 << DHT_PIN); }
void dht_set_input(void)  { PD_DDR &= ~(1 << DHT_PIN); }
void dht_high(void)       { PD_ODR |= (1 << DHT_PIN); }
void dht_low(void)        { PD_ODR &= ~(1 << DHT_PIN); }
uint8_t dht_read_pin(void){ return (PD_IDR & (1 << DHT_PIN)) ? 1 : 0; }

int16_t dht_read_byte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t timeout = 10000;
        while (!dht_read_pin()) {
            if (--timeout == 0) return -1;
        }
        delay_us(40);
        byte <<= 1;
        if (dht_read_pin()) byte |= 1;
        timeout = 10000;
        while (dht_read_pin()) {
            if (--timeout == 0) return -1;
        }
    }
    return byte;
}

int8_t dht_read(uint8_t *humidity, uint8_t *temperature) {
    if (humidity == 0 || temperature == 0) return -1;
    uint8_t data[5] = {0,0,0,0,0};
    dht_set_output();
    dht_low();
    delay_ms(18);
    dht_high();
    delay_us(30);
    dht_set_input();
    delay_us(80);
    delay_us(80);
    for (uint8_t i = 0; i < 5; i++) {
        int16_t b = dht_read_byte();
        if (b < 0) return -1;
        data[i] = (uint8_t)b;
    }
    dht_set_output();
    dht_high();
    if ((uint8_t)(data[0]+data[1]+data[2]+data[3]) != data[4]) return -1;
    *humidity = data[0];
    *temperature = data[2];
    return 0;
}

void led_on(void)  { PD_ODR |= (1 << LED_PIN); }
void led_off(void) { PD_ODR &= ~(1 << LED_PIN); }

void alert_blink_fast(void) {
    for (uint8_t i = 0; i < 6; i++) {
        led_on(); delay_ms(100);
        led_off(); delay_ms(100);
    }
}

void main(void) {
    uint8_t humidity, temperature;

    CLK_CKDIVR = 0;
    PD_DDR |= (1 << LED_PIN);
    PD_CR1 |= (1 << LED_PIN);
    led_off();

    while (1) {
        delay_ms(2000);

        if (dht_read(&humidity, &temperature) == 0) {
            if (temperature > 30 || humidity < 30) {
                alert_blink_fast();   // bad condition — fast blink
            } else {
                led_on(); delay_ms(50); led_off();  // OK — quick heartbeat blink
            }
        } else {
            // sensor read failed — 2 quick flashes as error signal
            led_on(); delay_ms(50); led_off(); delay_ms(50);
            led_on(); delay_ms(50); led_off();
        }
    }
}
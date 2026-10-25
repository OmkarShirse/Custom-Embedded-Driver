# ESP32 Fast GPIO Driver (ESP-IDF, esp32 target)

Register-level GPIO via W1TS/W1TC. Scope: **GPIO only**, **ESP32 (original) only**.

## Layout
```
CMakeLists.txt
sdkconfig.defaults
components/gpio_fast/{CMakeLists.txt, gpio_fast.c, include/gpio_fast.h}
main/{CMakeLists.txt, main.c}
```

## Guarantees
| Op | Writes | Atomic |
|---|---|---|
| set / clear / write(pin), set_mask / clear_mask | 1 | yes |
| write_mask, toggle_mask | 2 (clear, then set) | no: brief mixed state on pins |
| toggle, toggle_mask (RMW) | read + 1-2 | no: single-owner pins, or use `*_atomic` (all writers must use it) |

Hot-path functions are `always_inline`; call them from `IRAM_ATTR` code if the flash cache may be disabled.

## Pins
GPIO 6-11 refused (flash). 20, 24, 28-31 do not exist, refused. 34-39 input-only, no pulls. Warns on 0,1,2,3,5,12,15 (strap/console) and 16,17 (PSRAM).
Bank 1 masks: bit0 = GPIO32 ... bit7 = GPIO39. Use `1ULL<<n` with `config_mask` for pins >= 32.

## Benchmark
`main.c` prints ns per set+clear pair for this driver vs `gpio_set_level()`, interrupts off. Report your own numbers with IDF version, `-O2`, 240 MHz.
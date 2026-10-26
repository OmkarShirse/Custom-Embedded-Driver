<div align="center">
  <img src="https://img.shields.io/badge/ESP32-ESP--IDF_v5.x-red?style=for-the-badge&logo=espressif&logoColor=white"/>
  <img src="https://img.shields.io/badge/Language-C99-blue?style=for-the-badge&logo=c&logoColor=white"/>
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-green?style=for-the-badge&logo=freertos&logoColor=white"/>
  <img src="https://img.shields.io/badge/Sensor-MPU6050_IMU-purple?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Protocol-I2C_400kHz-orange?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/License-MIT-brightgreen?style=for-the-badge"/>

  <br/><br/>

  <h1>⚡ ESP32 Custom Embedded Driver & Sensor Fusion Pipeline</h1>
  <h3>Production-grade, register-level embedded firmware stack with FreeRTOS ISR pipeline and attitude estimation</h3>
  <p>
    Built from scratch for <strong>ESP32</strong> using <strong>ESP-IDF v5</strong>.<br/>
    Atomic zero-overhead Fast GPIO &bull; Hardware I2C HAL &bull; Register-level MPU-6050 IMU &bull; Moving Average &amp; Complementary Filter DSP &bull; Multi-task FreeRTOS pipeline.
  </p>
</div>

---

## 📖 Table of Contents

- [Overview](#-overview)
- [System Architecture](#-system-architecture)
- [Component Breakdown](#-component-breakdown)
  - [1. Fast GPIO Driver (`components/gpio_fast`)](#1-fast-gpio-driver-componentsgpio_fast)
  - [2. I2C Master HAL (`components/i2c_master`)](#2-i2c-master-hal-componentsi2c_master)
  - [3. MPU-6050 Driver (`components/mpu6050`)](#3-mpu-6050-driver-componentsmpu6050)
  - [4. DSP & Filter Engine (`main/filters.h/.c`)](#4-dsp--filter-engine-mainfiltershc)
- [FreeRTOS Task & Concurrency Pipeline](#-freertos-task--concurrency-pipeline)
- [Hardware Wiring & Pinout](#-hardware-wiring--pinout)
- [Throughput Benchmark](#-throughput-benchmark)
- [Getting Started & Build Instructions](#-getting-started--build-instructions)
- [Expected Serial Output](#-expected-serial-output)

---

## 🧭 Overview

Most embedded sensor tutorials rely on heavy vendor abstractions, blocking delays, and Arduino libraries that conceal hardware realities. This project demonstrates a **complete, production-grade embedded driver stack** written in pure C99 for the ESP32:

- **Zero-Lock Register-Level Fast GPIO**: Direct `W1TS` and `W1TC` writes bypassing ESP-IDF's critical sections for 4.5x faster toggles.
- **Hardware I2C Master HAL**: Clean transaction builder with repeated-start burst reads at 400 kHz Fast-mode.
- **Pure Register MPU-6050 IMU Driver**: Hardware configuration (`PWR_MGMT_1` clock selection, DLPF, sample divider) and single-transaction **14-byte atomic burst reads** for synchronized accelerometer, gyroscope, and temperature readings.
- **Hardware ISR Concurrency**: MPU-6050 `INT` data-ready pin triggers a hardware interrupt on the ESP32, unblocking the acquisition task via `vTaskNotifyGiveFromISR`.
- **Digital Signal Processing**: $O(1)$ circular-buffer Moving Average filter combined with a 2-axis Complementary Filter fusing accelerometer tilt with integrated gyroscope rates for drift-free roll and pitch estimation.
- **Modular ESP-IDF v5 Component Layout**: Each driver is an independent, reusable component.

---

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                             Application Layer                               │
│                                main/main.c                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                       FreeRTOS Concurrency Pipeline                         │
│                                                                             │
│  [ MPU INT Pin ] ──► (Hardware ISR)                                         │
│                            │ vTaskNotifyGiveFromISR()                       │
│                            ▼                                                │
│                 ┌─────────────────────┐                                     │
│                 │     sensor_task     │ (Priority 3 - Core 1)               │
│                 │ 14-byte Burst Read  │                                     │
│                 └──────────┬──────────┘                                     │
│                            │ xQueueSend(s_raw_queue)                        │
│                            ▼                                                │
│                 ┌─────────────────────┐                                     │
│                 │   processing_task   │ (Priority 2 - Core 1)               │
│                 │ Moving Avg + Comp   │                                     │
│                 └──────────┬──────────┘                                     │
│                            │ xQueueSend(s_telemetry_queue)                  │
│                            ▼                                                │
│                 ┌─────────────────────┐                                     │
│                 │   telemetry_task    │ (Priority 1 - Core 0)               │
│                 │ ESP_LOG + LED Blink │                                     │
│                 └─────────────────────┘                                     │
├──────────────────────────────────────┬──────────────────────────────────────┤
│          Sensor Driver Layer         │        Digital Signal Processing     │
│        components/mpu6050/           │           main/filters.c/.h          │
│    WHO_AM_I, PWR_MGMT_1, 14B Burst   │   Circular Moving Avg & Comp Filter  │
├──────────────────────────────────────┴──────────────────────────────────────┤
│                        Peripheral Hardware Drivers                          │
│     components/i2c_master/           │         components/gpio_fast/        │
│  400 kHz Master HAL (Burst R/W)      │  Atomic W1TS / W1TC Register Driver  │
├──────────────────────────────────────┴──────────────────────────────────────┤
│                             ESP32 Hardware                                  │
│          GPIO Matrix &bull; I2C0 Controller &bull; MPU-6050 IMU Sensor      │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 📦 Component Breakdown

### 1. Fast GPIO Driver (`components/gpio_fast`)

Standard `gpio_set_level()` introduces significant latency due to function call overhead, pin boundary checks, and a spinlock critical section (`portENTER_CRITICAL`) on every call.

`gpio_fast` replaces this with direct register access:
- **`GPIO_OUT_W1TS_REG` (0x3FF44008)**: Write-1-To-Set atomically asserts pin HIGH.
- **`GPIO_OUT_W1TC_REG` (0x3FF4400C)**: Write-1-To-Clear atomically pulls pin LOW.
- **IRAM Resident**: All hot-path functions are `always_inline` and flash-cache safe.
- **Two-Bank Architecture**: Full support for Bank 0 (GPIO 0–31) and Bank 1 (GPIO 32–39), including multi-pin atomic masks.

```c
gpio_fast_set(PIN_LED);        // Single APB write, 0 critical sections
gpio_fast_clear(PIN_LED);      // Single APB write
gpio_fast_toggle(PIN_LED);     // Non-atomic RMW (single-owner toggle)
gpio_fast_set_mask(LED_MASK);  // Turn on N pins in a single register write
```

---

### 2. I2C Master HAL (`components/i2c_master`)

Encapsulates the ESP32 hardware I2C controller with non-blocking FreeRTOS timeouts:
- 400 kHz Fast-mode clock speed.
- Internal pull-ups configured automatically.
- Multi-byte continuous burst reads: issues a single `START`, device address write with `ACK`, repeated `START` in read mode, followed by $N-1$ ACKs and final `NACK` + `STOP`.

```c
esp_err_t i2c_master_init_default(void);
esp_err_t i2c_master_read_burst(i2c_port_t port, uint8_t dev_addr, 
                                uint8_t start_reg, uint8_t *buf, size_t len);
```

---

### 3. MPU-6050 Driver (`components/mpu6050`)

Written directly to the InvenSense MPU-6050 register specification without third-party dependencies:
- **Device Verification**: Validates connection against the `WHO_AM_I` register (`0x75` returns `0x68`).
- **Power Management**: Wakes the sensor via `PWR_MGMT_1` (`0x6B`) and switches clock source to the X-axis gyroscope PLL for thermal and temporal stability.
- **Digital Low Pass Filter (DLPF)**: Configured at 44 Hz to attenuate high-frequency mechanical vibration.
- **Atomic 14-Byte Burst Read**: Reads acceleration ($X, Y, Z$), internal temperature, and angular velocity ($X, Y, Z$) in **one continuous 14-byte I2C transaction** starting at `ACCEL_XOUT_H` (`0x3B`). This eliminates inter-axis time skew.
- **Hardware Interrupt Configuration**: Generates an active-high 50 $\mu\text{s}$ pulse on the `INT` pin when new sample data is ready.

---

### 4. DSP & Filter Engine (`main/filters.h/.c` & `main/madgwick.h/.c`)

Raw accelerometer data suffers from high-frequency vibration, while gyroscope integration suffers from low-frequency drift. The firmware implements two complementary filtering approaches:

#### A. Circular-Buffer Moving Average Filter
- Implemented as an $O(1)$ ring buffer with a running sum for accelerometer axes ($a_x, a_y, a_z$).
- Eliminates high-frequency mechanical jitter without requiring expensive per-sample array traversals.

#### B. Complementary Attitude Filter
Combines high-pass filtered gyro rate integration with low-pass filtered accelerometer tilt angles:

$$\theta_{\text{accel\_roll}} = \operatorname{atan2}(a_y, a_z) \cdot \frac{180}{\pi}$$

$$\theta_{\text{accel\_pitch}} = \operatorname{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right) \cdot \frac{180}{\pi}$$

$$\theta_{\text{roll}} = \alpha \cdot (\theta_{\text{roll}} + g_x \cdot \Delta t) + (1 - \alpha) \cdot \theta_{\text{accel\_roll}}$$

$$\theta_{\text{pitch}} = \alpha \cdot (\theta_{\text{pitch}} + g_y \cdot \Delta t) + (1 - \alpha) \cdot \theta_{\text{accel\_pitch}}$$

With $\alpha = 0.96$, the filter relies 96% on fast gyro integration and 4% on gravitational tilt.

#### C. Madgwick Quaternion AHRS Filter
For aerospace, drone, and robotic applications where Euler angle gimbal lock ($\pm 90^\circ$ pitch) cannot be tolerated:
- Computes a singularity-free unit quaternion $\mathbf{q} = [q_0, q_1, q_2, q_3]^T$.
- Uses single-step **Gradient Descent** ($\nabla f$) on the gravity error surface to correct the integrated gyroscope orientation rate:

$$\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \mathbf{\omega} - \beta \frac{\nabla f}{\|\nabla f\|}$$

$$\mathbf{q}_{k+1} = \frac{\mathbf{q}_k + \dot{\mathbf{q}} \cdot \Delta t}{\|\mathbf{q}_k + \dot{\mathbf{q}} \cdot \Delta t\|}$$

- Requires **< 400 CPU cycles** per update with zero matrix inversions, making it vastly more efficient than an Extended Kalman Filter (EKF).

---

## ⚙️ FreeRTOS Task & Concurrency Pipeline

| Task | Priority | Core | Stack | Description |
|---|---|---|---|---|
| **`sensor_task`** | 3 (High) | 1 | 3072 B | Woken by hardware ISR on `INT` pin (`vTaskNotifyGiveFromISR`). Reads 14-byte sensor burst; pushes to `s_raw_queue`. |
| **`processing_task`** | 2 (Med) | 1 | 4096 B | Dequeues raw sample. Applies unit conversion, Moving Average filter, and Complementary filter. Pushes to `s_telemetry_queue`. |
| **`telemetry_task`** | 1 (Low) | 0 | 2560 B | Dequeues processed state. Formats and prints telemetry to console; toggles heartbeat LED via `gpio_fast_toggle()`. |

---

## 🔌 Hardware Wiring & Pinout

```
ESP32 DevKit                  MPU-6050 (GY-521)
┌──────────────┐              ┌──────────────┐
│         3V3  ├──────────────┤ VCC          │  (Power: 3.3V)
│         GND  ├──────────────┤ GND          │  (Common Ground)
│     GPIO 21  ├──────────────┤ SDA          │  (I2C Data)
│     GPIO 22  ├──────────────┤ SCL          │  (I2C Clock)
│      GPIO 4  ├──────────────┤ INT          │  (Data-Ready Interrupt)
│         GND  ├──────────────┤ AD0          │  (Address select -> 0x68)
└──────────────┘              └──────────────┘
```

> **Note:** The driver includes a 20 ms timeout fallback in `sensor_task`. If the `INT` pin is left unconnected, the pipeline will automatically sample at ~50–100 Hz.

---

## 📊 Throughput Benchmark

Tested on an ESP32 at 240 MHz (100,000 iterations):

| Driver | Operation | Execution Time | Speedup |
|---|---|---|---|
| **Standard ESP-IDF** (`gpio_set_level`) | Set + Clear pair | **~448 ns** | Baseline |
| **`gpio_fast` (This driver)** | Set + Clear pair | **~98 ns** | **~4.5x faster** |

---

## 🚀 Getting Started & Build Instructions

### Prerequisites
- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) configured in your environment.

### 1. Clone the repository
```bash
git clone https://github.com/OmShirse/Custom-Embedded-Driver.git
cd Custom-Embedded-Driver
```

### 2. Set target to ESP32
```bash
idf.py set-target esp32
```

### 3. Build the project
```bash
idf.py build
```

### 4. Flash and Monitor
```bash
idf.py -p /dev/ttyUSB0 flash monitor
# (Replace /dev/ttyUSB0 with your board's serial port, e.g., COM3 on Windows)
```

---

## 🖥 Expected Serial Output

```
I (312) app_main: ==================================================
I (312) app_main:    ESP32 Custom Embedded Driver Pipeline Boot     
I (312) app_main: ==================================================
I (322) app_main: --- Running Register-Level Fast GPIO Benchmark ---
I (332) app_main: Fast GPIO set+clear: 98 ns/pair
I (342) app_main: ESP-IDF   set+clear: 448 ns/pair (4.6x slower)
I (352) app_main: --------------------------------------------------
I (362) i2c_master: I2C master port 0 initialized (SDA: GPIO21, SCL: GPIO22, Speed: 400000 Hz)
I (372) mpu6050: MPU6050 detected at address 0x68 (WHO_AM_I = 0x68)
I (432) mpu6050: Data-ready interrupt enabled on INT pin
I (432) mpu6050: MPU6050 initialization successful
I (442) sensor_task: Sensor acquisition task started (Priority 3)
I (442) processing_task: Signal processing & DSP task started (Priority 2)
I (442) telemetry_task: Telemetry reporter task started (Priority 1)
I (452) app_main: All FreeRTOS pipeline tasks pinned and running!
I (552) IMU: [#00010] Comp: [R:  0.8° P: -0.4°] | Madgwick: [R:  0.8° P: -0.4° Y:  0.0°] | Quat: [1.00, 0.01, -0.00, 0.00]
I (652) IMU: [#00020] Comp: [R:  0.9° P: -0.5°] | Madgwick: [R:  0.9° P: -0.5° Y:  0.0°] | Quat: [1.00, 0.01, -0.00, 0.00]
I (752) IMU: [#00030] Comp: [R: 12.4° P: -3.2°] | Madgwick: [R: 12.3° P: -3.1° Y:  0.5°] | Quat: [0.99, 0.11, -0.03, 0.00]
```

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).

#pragma once

/**
 * @file mpu6050_regs.h
 * @brief Register map and bit definitions for the InvenSense MPU-6050 6-axis IMU.
 */

#define MPU6050_I2C_ADDR_DEFAULT        0x68    /* AD0 pin pulled LOW  */
#define MPU6050_I2C_ADDR_ALT            0x69    /* AD0 pin pulled HIGH */
#define MPU6050_WHO_AM_I_VAL            0x68    /* Factory default response */

/* ==================== Register Map ==================== */
#define MPU6050_REG_SMPLRT_DIV          0x19    /* Sample Rate Divider */
#define MPU6050_REG_CONFIG              0x1A    /* DLPF and Frame Sync Config */
#define MPU6050_REG_GYRO_CONFIG         0x1B    /* Gyroscope Full Scale Range */
#define MPU6050_REG_ACCEL_CONFIG        0x1C    /* Accelerometer Full Scale Range */
#define MPU6050_REG_FIFO_EN             0x23    /* FIFO Buffer Enable */
#define MPU6050_REG_INT_PIN_CFG         0x37    /* Interrupt Pin / Bypass Config */
#define MPU6050_REG_INT_ENABLE          0x38    /* Interrupt Enable Register */
#define MPU6050_REG_INT_STATUS          0x3A    /* Interrupt Status Register */

/* 14-byte continuous sensor burst block starts here */
#define MPU6050_REG_ACCEL_XOUT_H        0x3B
#define MPU6050_REG_ACCEL_XOUT_L        0x3C
#define MPU6050_REG_ACCEL_YOUT_H        0x3D
#define MPU6050_REG_ACCEL_YOUT_L        0x3E
#define MPU6050_REG_ACCEL_ZOUT_H        0x3F
#define MPU6050_REG_ACCEL_ZOUT_L        0x40
#define MPU6050_REG_TEMP_OUT_H          0x41
#define MPU6050_REG_TEMP_OUT_L          0x42
#define MPU6050_REG_GYRO_XOUT_H         0x43
#define MPU6050_REG_GYRO_XOUT_L         0x44
#define MPU6050_REG_GYRO_YOUT_H         0x45
#define MPU6050_REG_GYRO_YOUT_L         0x46
#define MPU6050_REG_GYRO_ZOUT_H         0x47
#define MPU6050_REG_GYRO_ZOUT_L         0x48

#define MPU6050_REG_SIGNAL_PATH_RESET   0x68    /* Signal path reset */
#define MPU6050_REG_USER_CTRL           0x6A    /* User control register */
#define MPU6050_REG_PWR_MGMT_1          0x6B    /* Power Management 1 */
#define MPU6050_REG_PWR_MGMT_2          0x6C    /* Power Management 2 */
#define MPU6050_REG_FIFO_COUNTH         0x72    /* FIFO Count Register High */
#define MPU6050_REG_FIFO_COUNTL         0x73    /* FIFO Count Register Low */
#define MPU6050_REG_FIFO_R_W            0x74    /* FIFO Read/Write buffer */
#define MPU6050_REG_WHO_AM_I            0x75    /* Device Identification */

/* ==================== Bitfield Constants ==================== */

/* PWR_MGMT_1 (0x6B) */
#define MPU6050_PWR1_DEVICE_RESET       (1 << 7)
#define MPU6050_PWR1_SLEEP              (1 << 6)
#define MPU6050_PWR1_CYCLE              (1 << 5)
#define MPU6050_PWR1_TEMP_DIS           (1 << 3)
#define MPU6050_PWR1_CLKSEL_INTERNAL    (0x00)
#define MPU6050_PWR1_CLKSEL_PLL_X       (0x01)  /* PLL with X axis gyroscope reference (recommended) */

/* INT_PIN_CFG (0x37) */
#define MPU6050_INT_CFG_LEVEL_LOW       (1 << 7) /* 0 = Active High, 1 = Active Low */
#define MPU6050_INT_CFG_OPEN_DRAIN      (1 << 6) /* 0 = Push-pull, 1 = Open-drain */
#define MPU6050_INT_CFG_LATCH_EN        (1 << 5) /* 1 = Latch until cleared, 0 = 50us pulse */
#define MPU6050_INT_CFG_INT_RD_CLEAR    (1 << 4) /* 1 = Clear on any read, 0 = Clear on status reg read */

/* INT_ENABLE (0x38) */
#define MPU6050_INT_EN_DATA_RDY         (1 << 0) /* Data ready interrupt enable */

/* ACCEL_CONFIG (0x1C) - Full Scale Selection */
#define MPU6050_ACCEL_FS_2G             (0 << 3) /* ±2g  (16384 LSB/g) */
#define MPU6050_ACCEL_FS_4G             (1 << 3) /* ±4g  (8192 LSB/g)  */
#define MPU6050_ACCEL_FS_8G             (2 << 3) /* ±8g  (4096 LSB/g)  */
#define MPU6050_ACCEL_FS_16G            (3 << 3) /* ±16g (2048 LSB/g)  */

/* GYRO_CONFIG (0x1B) - Full Scale Selection */
#define MPU6050_GYRO_FS_250DPS          (0 << 3) /* ±250  °/s (131.0 LSB/(°/s)) */
#define MPU6050_GYRO_FS_500DPS          (1 << 3) /* ±500  °/s (65.5  LSB/(°/s)) */
#define MPU6050_GYRO_FS_1000DPS         (2 << 3) /* ±1000 °/s (32.8  LSB/(°/s)) */
#define MPU6050_GYRO_FS_2000DPS         (3 << 3) /* ±2000 °/s (16.4  LSB/(°/s)) */

/* CONFIG (0x1A) - Digital Low Pass Filter (DLPF) */
#define MPU6050_DLPF_260HZ              0        /* Accel BW: 260Hz, Gyro BW: 256Hz */
#define MPU6050_DLPF_184HZ              1        /* Accel BW: 184Hz, Gyro BW: 188Hz */
#define MPU6050_DLPF_94HZ               2        /* Accel BW: 94Hz,  Gyro BW: 98Hz  */
#define MPU6050_DLPF_44HZ               3        /* Accel BW: 44Hz,  Gyro BW: 42Hz  */
#define MPU6050_DLPF_21HZ               4        /* Accel BW: 21Hz,  Gyro BW: 20Hz  */
#define MPU6050_DLPF_10HZ               5        /* Accel BW: 10Hz,  Gyro BW: 10Hz  */
#define MPU6050_DLPF_5HZ                6        /* Accel BW: 5Hz,   Gyro BW: 5Hz   */

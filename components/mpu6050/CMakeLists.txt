idf_component_register(
    SRCS "mpu6050.c"
    INCLUDE_DIRS "include"
    REQUIRES i2c_master esp_common
    PRIV_REQUIRES log
)

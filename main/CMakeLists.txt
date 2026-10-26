idf_component_register(
    SRCS "main.c" "filters.c"
    INCLUDE_DIRS "."
    REQUIRES gpio_fast i2c_master mpu6050 esp_timer
    PRIV_REQUIRES log driver
)

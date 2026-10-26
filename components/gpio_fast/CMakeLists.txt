idf_component_register(
    SRCS "gpio_fast.c"
    INCLUDE_DIRS "include"
    REQUIRES soc esp_common freertos
    PRIV_REQUIRES esp_rom log driver
)

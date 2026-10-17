idf_component_register(
    SRCS "encoders.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio motors
)

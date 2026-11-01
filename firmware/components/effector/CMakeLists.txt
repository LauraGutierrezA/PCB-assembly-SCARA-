idf_component_register(
    SRCS "effector.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc motors
    PRIV_REQUIRES esp_driver_gpio
)

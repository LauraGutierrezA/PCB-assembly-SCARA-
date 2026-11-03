idf_component_register(
    SRCS "observers.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES freertos encoders motors homing_pro control
)

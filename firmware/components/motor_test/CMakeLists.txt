idf_component_register(
    SRCS "motor_test.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES motors homing_pro freertos
)

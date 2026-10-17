idf_component_register(
    SRCS "homing_pro.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES motors encoders freertos
)

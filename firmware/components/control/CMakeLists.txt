idf_component_register(
    SRCS "control.c"
    INCLUDE_DIRS "include"
    REQUIRES kinematics
    PRIV_REQUIRES freertos
)

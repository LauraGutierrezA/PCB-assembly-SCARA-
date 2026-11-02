idf_component_register(
    SRCS "control.c" "controlRST.c" "profile.c"
    INCLUDE_DIRS "include"
    REQUIRES kinematics
    PRIV_REQUIRES freertos encoders motors homing_pro motor_test
)

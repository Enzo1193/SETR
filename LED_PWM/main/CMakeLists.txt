idf_component_register(
    SRCS "ledc_basic_example_main.c" "rgb_control.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos
)

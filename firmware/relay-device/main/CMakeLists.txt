idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES cc1101 driver nvs_flash rf_security
)

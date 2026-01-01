# set(COMPONENT_SRCS "main.c")
# set(COMPONENT_ADD_INCLUDEDIRS "")

# register_component()

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_matter cc1101 driver nvs_flash rf_security
)
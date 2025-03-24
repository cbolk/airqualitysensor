idf_component_register(
        SRCS
            "test_pms5003_main.c"
            "pms5003.c"

        INCLUDE_DIRS
            "."
        REQUIRES esp_driver_uart esp_driver_gpio
)
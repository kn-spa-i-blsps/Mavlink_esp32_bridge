   idf_component_register(SRCS "main.c"
                          INCLUDE_DIRS "."
                          REQUIRES mavlink
                          PRIV_REQUIRES esp_driver_uart esp_timer)

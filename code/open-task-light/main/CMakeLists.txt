idf_component_register(SRCS "otl_main.c"
                      INCLUDE_DIRS "."
                      REQUIRES led_strip esp_timer esp_adc driver
                               esp_driver_gpio esp_driver_ledc esp_driver_touch_sens
                               esp_driver_tsens esp_driver_uart otl_circadian)

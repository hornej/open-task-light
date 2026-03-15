idf_component_register(SRCS "otl_main.c"
                      INCLUDE_DIRS "."
                      REQUIRES led_strip esp_timer esp_adc driver otl_circadian)

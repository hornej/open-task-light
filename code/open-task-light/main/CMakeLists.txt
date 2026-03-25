set(otl_main_srcs
    "otl_main.c"
)

set(otl_main_requires
    led_strip
    esp_timer
    esp_adc
    driver
    esp_driver_gpio
    esp_driver_ledc
    esp_driver_touch_sens
    esp_driver_tsens
    esp_driver_uart
    otl_circadian
    otl_net
)

set(otl_main_priv_requires
    esp_app_format
)

if(CONFIG_OTL_HA_MQTT_ENABLE)
    list(APPEND otl_main_srcs "otl_mqtt.c")
    list(APPEND otl_main_requires mqtt)
endif()

idf_component_register(
    SRCS ${otl_main_srcs}
    INCLUDE_DIRS "."
    REQUIRES ${otl_main_requires}
    PRIV_REQUIRES ${otl_main_priv_requires}
)

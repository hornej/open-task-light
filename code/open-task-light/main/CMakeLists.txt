set(otl_main_srcs
    "otl_main.c"
)

set(otl_main_include_dirs
    "."
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

if(CONFIG_OTL_HOMEKIT_ENABLE)
    list(APPEND otl_main_srcs "otl_homekit.c")
    list(APPEND otl_main_priv_requires esp_hap_core esp_hap_apple_profiles)
    list(APPEND otl_main_include_dirs
        "${PROJECT_DIR}/components/esp-homekit-sdk/components/homekit/esp_hap_core/include"
        "${PROJECT_DIR}/components/esp-homekit-sdk/components/homekit/esp_hap_apple_profiles/include"
    )
endif()

idf_component_register(
    SRCS ${otl_main_srcs}
    INCLUDE_DIRS ${otl_main_include_dirs}
    REQUIRES ${otl_main_requires}
    PRIV_REQUIRES ${otl_main_priv_requires}
)

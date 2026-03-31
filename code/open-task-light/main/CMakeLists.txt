set(otl_main_srcs
    "otl_main.c"
)

set(OTL_BUILD_INFO_HEADER
    "${CMAKE_CURRENT_BINARY_DIR}/otl_build_info.h"
)

set(otl_main_include_dirs
    "."
    "${CMAKE_CURRENT_BINARY_DIR}"
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
    nvs_flash
    esp_wifi
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

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    add_custom_target(otl_refresh_build_info ALL
        COMMAND ${CMAKE_COMMAND}
                -DOUTPUT_FILE=${OTL_BUILD_INFO_HEADER}
                -P "${CMAKE_CURRENT_LIST_DIR}/generate_build_info.cmake"
        COMMENT "Generating Open Task Light build info"
        VERBATIM
        BYPRODUCTS "${OTL_BUILD_INFO_HEADER}"
    )
endif()

idf_component_register(
    SRCS ${otl_main_srcs}
    INCLUDE_DIRS ${otl_main_include_dirs}
    REQUIRES ${otl_main_requires}
    PRIV_REQUIRES ${otl_main_priv_requires}
)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    add_dependencies(${COMPONENT_LIB} otl_refresh_build_info)
endif()

if(CONFIG_OTL_HOMEKIT_ENABLE)
    target_link_libraries(${COMPONENT_LIB} PRIVATE
        idf::esp_hap_core
        idf::esp_hap_apple_profiles
    )
endif()

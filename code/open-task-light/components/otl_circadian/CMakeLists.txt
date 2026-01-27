idf_component_register(
    SRCS "otl_circadian.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_event esp_netif esp_wifi lwip nvs_flash
)


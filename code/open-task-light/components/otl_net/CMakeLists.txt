idf_component_register(
    SRCS "otl_net.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_event esp_netif esp_wifi nvs_flash
)

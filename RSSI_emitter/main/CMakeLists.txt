idf_component_register(
    SRCS "RSSI_emitter_main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_event esp_netif
)
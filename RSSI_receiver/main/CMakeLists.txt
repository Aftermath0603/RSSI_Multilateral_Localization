idf_component_register(
    SRCS "RSSI_receiver_main.c"
    INCLUDE_DIRS "."
    REQUIRES driver bt nvs_flash esp_wifi esp_event esp_timer freertos
)
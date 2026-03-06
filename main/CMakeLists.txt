idf_component_register(
    SRCS "main.c" "inference.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_psram
        esp_system
        driver
        app_update
        esp_partition
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        lwip
        mbedtls
        esp_http_server
        esp_http_client
        esp_https_ota
        mqtt
        esp32-camera
        mdns
        esp-dl
        human_face_detect
        esp_driver_sdmmc
        sdmmc
        fatfs
)

idf_component_register( SRCS "THN128.cpp" "heltec_wsl.cpp"
                        INCLUDE_DIRS "." "../../shared/"
                        REQUIRES tools lora sensors nvs_flash esp_driver_rmt esp_http_client vfs mbedtls esp_driver_uart)

set(requires esp_driver_spi spi_flash esp_driver_i2c esp_driver_gpio esp_adc esp_wifi)

idf_component_register(SRCS "hauptwetterstation.cpp" "../../shared/u8g2/u8g2_esp32_hal.c"
                    INCLUDE_DIRS "." "../../shared/" "../../shared/lora"
                    REQUIRES ${requires}
                    )

idf_build_set_property(CXX_COMPILE_OPTIONS "-Wno-volatile" APPEND)

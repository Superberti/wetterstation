set(component_srcs "lora.cpp" "")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver" "tools"
                       REQUIRES esp_driver_spi esp_driver_gpio)



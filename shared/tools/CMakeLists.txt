set(component_srcs "tools.cpp" "u8g2_esp32_hal.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "." "../u8g2/csrc"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")

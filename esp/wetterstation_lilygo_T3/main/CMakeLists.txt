idf_component_register(SRCS "lora_temp.cpp" "../../shared/u8g2/u8g2_esp32_hal.c" 
                    INCLUDE_DIRS "." "../../shared/")
idf_build_set_property(CXX_COMPILE_OPTIONS "-Wno-volatile" APPEND)
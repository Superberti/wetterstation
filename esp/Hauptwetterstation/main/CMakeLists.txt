#idf_component_register(SRCS "hauptwetterstation.cpp" "../../shared/u8g2/u8g2_esp32_hal.c" INCLUDE_DIRS "." "../../shared/")
set(requires "")

idf_component_register(SRCS "hauptwetterstation.cpp" "../../shared/u8g2/u8g2_esp32_hal.c"
                    INCLUDE_DIRS "." "../../shared/"
                    REQUIRES ${requires}
                    )

idf_build_set_property(CXX_COMPILE_OPTIONS "-Wno-volatile" APPEND)

set(component_srcs "bmp280.cpp" "bmp390.cpp" "sht35.cpp" "veml7700.cpp" "sht40.cpp" "ads1015.cpp" "ds3231.cpp")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")

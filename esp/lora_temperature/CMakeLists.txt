# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
#set(EXTRA_COMPONENT_DIRS components/ssd1306)
set(EXTRA_COMPONENT_DIRS "../shared/u8g2" "../shared/tools" "../shared/lora" "../shared/sensors")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lora_temperature)
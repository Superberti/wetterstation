idf_component_register(SRCS "wetterstation.cpp" "wetterstation-lora.cpp" "bmp280.cpp" "veml7700.cpp" "sht35.cpp" "esp32_digital_led_lib.cpp"
    "tools.cpp" "cbor_tools.cpp" "../components/cbor/cborencoder_close_container_checked.c" "../components/lora/lora.c"
    "../components/cbor/cborencoder.c" "../components/cbor/cborerrorstrings.c" "../components/cbor/cborparser_dup_string.c"
    "../components/cbor/cborparser.c" "../components/cbor/cborpretty_stdio.c" "../components/cbor/cborpretty.c" "../components/cbor/cbortojson.c"
    "../components/cbor/cborvalidation.c" "../components/cbor/open_memstream.c"
                    INCLUDE_DIRS ".")

idf_component_register(SRCS "peripheral.cpp" "mcp2515.cpp" "st7735.cpp" "barometer.cpp" "ble.cpp" "staging.c"
 INCLUDE_DIRS "../../inc" REQUIRES driver esp_timer esp_adc esp_system bt nvs_flash bmp3 image_staging esp_partition)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
if(EPPG_QEMU)
 target_compile_definitions(${COMPONENT_LIB} PUBLIC EPPG_QEMU=1)
endif()

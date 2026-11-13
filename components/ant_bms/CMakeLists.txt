idf_component_register(SRCS "BMS_CAN.cpp" INCLUDE_DIRS "." REQUIRES controller_platform)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)

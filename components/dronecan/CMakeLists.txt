file(GLOB DRONECAN_C "src/*.c")
idf_component_register(SRCS ${DRONECAN_C} "src/CanardAdapter.cpp" "src/SineEsc.cpp"
 INCLUDE_DIRS "src" REQUIRES controller_platform driver)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)

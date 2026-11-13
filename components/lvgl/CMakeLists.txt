file(GLOB_RECURSE LVGL_SOURCES "src/*.c" "src/*.cpp")
idf_component_register(SRCS ${LVGL_SOURCES} INCLUDE_DIRS "." "../../inc/sp140/lvgl")
target_compile_definitions(${COMPONENT_LIB} PUBLIC LV_CONF_INCLUDE_SIMPLE LV_LVGL_H_INCLUDE_SIMPLE)

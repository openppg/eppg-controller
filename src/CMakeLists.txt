file(GLOB_RECURSE SOURCES "sp140/*.cpp")

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS "../inc" "../inc/sp140" "../inc/sp140/lvgl"
                       REQUIRES arduino)

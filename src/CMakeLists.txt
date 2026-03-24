
add_subdirectory(core)

add_library(${AHOI_LIB} STATIC $<TARGET_OBJECTS:${CORE_LIB}>)
target_include_directories(${AHOI_LIB} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/core/include")
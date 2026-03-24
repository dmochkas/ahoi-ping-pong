
add_library(${CORE_LIB} OBJECT "src/core.c")
target_include_directories(${CORE_LIB} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
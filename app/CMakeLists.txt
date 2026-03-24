
set(PING_PONG_EXEC_NAME "ahoi-ping-pong")

add_executable(${PING_PONG_EXEC_NAME} "main.c")
target_compile_definitions(${PING_PONG_EXEC_NAME} PRIVATE AHOI_PORT="${AHOI_PORT}" AHOI_ID=${AHOI_ID} PING_RULE_ID=${PING_RULE_ID} PONG_RULE_ID=${PONG_RULE_ID})
target_link_libraries(${PING_PONG_EXEC_NAME} PRIVATE ${AHOI_LIB})
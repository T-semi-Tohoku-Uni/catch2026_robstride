cmake_minimum_required(VERSION 3.22)
project(motor_host_tests C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(board_protocol_test board_protocol_test.c ../Core/Src/board_protocol.c)
target_include_directories(board_protocol_test PRIVATE ../Core/Inc)
add_test(NAME board_protocol COMMAND board_protocol_test)

# Copy the public header so its quoted main.h resolves to the host HAL stub.
configure_file(../Core/Inc/cybergear.h include/cybergear.h COPYONLY)
configure_file(../Core/Inc/cybergear_homing.h include/cybergear_homing.h COPYONLY)
add_executable(homing_test homing_test.c ../Core/Src/cybergear_homing.c)
target_include_directories(homing_test PRIVATE stubs "${CMAKE_CURRENT_BINARY_DIR}/include" ../Core/Inc)
add_test(NAME homing COMMAND homing_test)

foreach(header motor_app.h robstride_app.h can_init.h)
    configure_file(../Core/Inc/${header} include/${header} COPYONLY)
endforeach()
# Include the actual application in the test translation unit so its static
# state is inspectable. The driver and wire codec remain production sources.
add_executable(motor_app_test motor_app_test.c
    ../Core/Src/robstride_app.c ../Core/Src/board_protocol.c)
target_include_directories(motor_app_test PRIVATE app_stubs
    "${CMAKE_CURRENT_BINARY_DIR}/include" ../Core/Inc)
foreach(scenario delayed-power transient-tx missing-feedback wrong-mode
        wrong-id stale-feedback stale-ready late-fault fault stop-retry
        persistent-stop tick-rollover silent-config-loss ignored-enable
        temporary-fault cg-adrc-failure timer-start-failure handoff-fault
        persistent-handoff-fault)
    add_test(NAME motor_app_${scenario} COMMAND motor_app_test ${scenario})
    set_tests_properties(motor_app_${scenario} PROPERTIES TIMEOUT 5)
endforeach()

foreach(target board_protocol_test homing_test motor_app_test)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /utf-8)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
        target_link_libraries(${target} PRIVATE m)
    endif()
endforeach()

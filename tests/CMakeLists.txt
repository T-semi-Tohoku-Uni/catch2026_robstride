cmake_minimum_required(VERSION 3.22)
project(cybergear_host_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
enable_testing()
if(MSVC)
    add_compile_options(/utf-8 /UNDEBUG)
endif()
option(CYBERGEAR_SANITIZERS "Require AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

add_executable(cybergear_host_tests
    test_main.c driver_test.c trajectory_test.c controller_test.c dynamics_test.c controller_simulation.c test_motion.c
    ../Core/Src/cybergear.c ../Core/Src/cybergear_controller.c
    ../Core/Src/cybergear_dynamics.c ../Core/Src/cybergear_trajectory.c ../Core/Src/cybergear_test_motion.c)
target_include_directories(cybergear_host_tests PRIVATE stubs ../Core/Inc)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(cybergear_host_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror -fno-fast-math -UNDEBUG)
    target_link_libraries(cybergear_host_tests PRIVATE m)
    if(CYBERGEAR_SANITIZERS)
        target_compile_options(cybergear_host_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(cybergear_host_tests PRIVATE -fsanitize=address,undefined)
    endif()
endif()
add_test(NAME cybergear_host_tests COMMAND cybergear_host_tests)

# Compile the real RobStride driver against a host HAL, suppressing the
# CubeMX main.h contents (GPIO/startup declarations are irrelevant here).
add_executable(robstride_tx_test robstride_tx_test.c ../Core/Src/robstride_app.c)
target_include_directories(robstride_tx_test PRIVATE stubs ../Core/Inc)
target_compile_definitions(robstride_tx_test PRIVATE __MAIN_H APP_CYBERGEAR_STANDALONE_TEST=1)
if(MSVC)
    target_compile_options(robstride_tx_test PRIVATE /UNDEBUG
        "/FI${CMAKE_CURRENT_SOURCE_DIR}/stubs/stm32g4xx_hal.h")
else()
    target_compile_options(robstride_tx_test PRIVATE -UNDEBUG
        -include "${CMAKE_CURRENT_SOURCE_DIR}/stubs/stm32g4xx_hal.h")
endif()
add_test(NAME robstride_tx_isolation COMMAND robstride_tx_test)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME cybergear_standalone_commands
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_standalone_commands.py"
        "${CMAKE_C_COMPILER}" "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME cybergear_real_tim6_scheduler
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_scheduler.py"
        "${CMAKE_C_COMPILER}" "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME cybergear_real_fdcan3_dispatch
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_rx_dispatch.py"
        "${CMAKE_C_COMPILER}" "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}")

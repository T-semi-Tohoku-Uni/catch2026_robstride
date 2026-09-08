cmake_minimum_required(VERSION 3.22)
project(cybergear_host_tests C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
enable_testing()
option(CYBERGEAR_SANITIZERS "Require AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

add_executable(cybergear_host_tests
    test_main.c driver_test.c trajectory_test.c controller_test.c dynamics_test.c controller_simulation.c calibration_test.c
    ../Core/Src/cybergear.c ../Core/Src/cybergear_controller.c
    ../Core/Src/cybergear_dynamics.c ../Core/Src/cybergear_trajectory.c ../Core/Src/cybergear_calibration.c)
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

add_executable(cybergear_calibration_app_tests
    calibration_app_test.c ../Core/Src/cybergear_calibration_app.c
    ../Core/Src/cybergear_calibration.c ../Core/Src/cybergear.c
    ../Core/Src/cybergear_controller.c ../Core/Src/cybergear_dynamics.c
    ../Core/Src/cybergear_trajectory.c)
target_include_directories(cybergear_calibration_app_tests PRIVATE stubs ../Core/Inc)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(cybergear_calibration_app_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror -fno-fast-math -UNDEBUG)
    target_link_libraries(cybergear_calibration_app_tests PRIVATE m)
    if(CYBERGEAR_SANITIZERS)
        target_compile_options(cybergear_calibration_app_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(cybergear_calibration_app_tests PRIVATE -fsanitize=address,undefined)
    endif()
endif()
add_test(NAME cybergear_calibration_app_tests COMMAND cybergear_calibration_app_tests)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME cybergear_real_tim6_scheduler
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_scheduler.py"
        "${CMAKE_C_COMPILER}" "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}")
add_test(NAME cybergear_real_fdcan3_dispatch
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_rx_dispatch.py"
        "${CMAKE_C_COMPILER}" "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}")

add_test(NAME cybergear_calibration_analysis
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_calibration_analysis.py")
add_test(NAME cybergear_calibration_log_pipeline
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_calibration_log_pipeline.py"
        $<TARGET_FILE:cybergear_calibration_app_tests>)
add_test(NAME cybergear_calibration_isolation
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_calibration_integration.py"
        "${CMAKE_C_COMPILER}" "${CMAKE_CURRENT_SOURCE_DIR}/.." "${CMAKE_CURRENT_BINARY_DIR}")

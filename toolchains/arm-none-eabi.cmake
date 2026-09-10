set(ARM_GNU_TOOLCHAIN_BIN_DIR "" CACHE PATH "Directory containing a complete Arm GNU Toolchain")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_GNU_TOOLCHAIN_BIN_DIR)

function(catch_arm_toolchain_valid compiler result)
    set(${result} FALSE PARENT_SCOPE)
    if(NOT EXISTS "${compiler}")
        return()
    endif()
    foreach(library nano.specs libc.a libm.a)
        execute_process(COMMAND "${compiler}" "-print-file-name=${library}"
            OUTPUT_VARIABLE library_path OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET RESULT_VARIABLE library_result TIMEOUT 10)
        if(NOT library_result EQUAL 0 OR NOT IS_ABSOLUTE "${library_path}" OR
                NOT EXISTS "${library_path}")
            return()
        endif()
    endforeach()
    execute_process(COMMAND "${compiler}" -mcpu=cortex-m4 -mfpu=fpv4-sp-d16
        -mfloat-abi=hard -fsyntax-only "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/check_standard_headers.c"
        ERROR_QUIET OUTPUT_QUIET RESULT_VARIABLE headers_result TIMEOUT 10)
    if(headers_result EQUAL 0)
        set(${result} TRUE PARENT_SCOPE)
    endif()
endfunction()

if(CMAKE_HOST_WIN32)
    set(arm_tool_suffix ".exe")
else()
    set(arm_tool_suffix "")
endif()

if(ARM_GNU_TOOLCHAIN_BIN_DIR)
    set(arm_compiler "${ARM_GNU_TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc${arm_tool_suffix}")
elseif(CMAKE_C_COMPILER OR DEFINED ENV{CC})
    if(CMAKE_C_COMPILER)
        set(arm_compiler_name "${CMAKE_C_COMPILER}")
    else()
        set(arm_compiler_name "$ENV{CC}")
    endif()
    find_program(arm_compiler NAMES "${arm_compiler_name}" NO_CACHE)
else()
    find_program(arm_path_compiler NAMES arm-none-eabi-gcc NO_CACHE)
    file(GLOB arm_cube_directories LIST_DIRECTORIES TRUE
        "/opt/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin"
        "/opt/st/stm32cubeclt_*/GNU-tools-for-STM32/bin"
        "C:/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin")
    list(SORT arm_cube_directories COMPARE NATURAL ORDER DESCENDING)
    set(arm_candidates "${arm_path_compiler}")
    foreach(directory IN LISTS arm_cube_directories)
        list(APPEND arm_candidates "${directory}/arm-none-eabi-gcc${arm_tool_suffix}")
    endforeach()
    foreach(candidate IN LISTS arm_candidates)
        catch_arm_toolchain_valid("${candidate}" candidate_valid)
        if(candidate_valid)
            set(arm_compiler "${candidate}")
            break()
        endif()
    endforeach()
endif()

catch_arm_toolchain_valid("${arm_compiler}" arm_compiler_valid)
if(NOT arm_compiler_valid)
    message(FATAL_ERROR
        "A complete Arm GNU Toolchain with standard C headers, libc, libm and nano.specs is required. "
        "Selected compiler: '${arm_compiler}'. "
        "Set -DARM_GNU_TOOLCHAIN_BIN_DIR=/path/to/GNU-tools-for-STM32/bin and reconfigure.")
endif()
get_filename_component(arm_bin_dir "${arm_compiler}" DIRECTORY)
get_filename_component(arm_bin_dir "${arm_bin_dir}" ABSOLUTE)
set(ARM_GNU_TOOLCHAIN_BIN_DIR "${arm_bin_dir}" CACHE PATH
    "Directory containing a complete Arm GNU Toolchain" FORCE)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/gcc-arm-none-eabi.cmake")

set(TOOLCHAIN_PREFIX "${ARM_GNU_TOOLCHAIN_BIN_DIR}/arm-none-eabi-")
foreach(tool_pair IN ITEMS "C_COMPILER|gcc" "ASM_COMPILER|gcc" "CXX_COMPILER|g++"
        "LINKER|g++" "OBJCOPY|objcopy" "SIZE|size" "AR|ar" "RANLIB|ranlib"
        "C_COMPILER_AR|gcc-ar" "C_COMPILER_RANLIB|gcc-ranlib"
        "CXX_COMPILER_AR|gcc-ar" "CXX_COMPILER_RANLIB|gcc-ranlib"
        "ASM_COMPILER_AR|gcc-ar" "ASM_COMPILER_RANLIB|gcc-ranlib")
    string(REPLACE "|" ";" tool_pair "${tool_pair}")
    list(GET tool_pair 0 variable_suffix)
    list(GET tool_pair 1 executable_suffix)
    set(tool_path "${TOOLCHAIN_PREFIX}${executable_suffix}${arm_tool_suffix}")
    if(NOT EXISTS "${tool_path}")
        message(FATAL_ERROR "Missing Arm GNU tool: ${tool_path}")
    endif()
    set(CMAKE_${variable_suffix} "${tool_path}")
    set(CMAKE_${variable_suffix} "${tool_path}" CACHE FILEPATH "Arm GNU ${executable_suffix}" FORCE)
endforeach()

#-- Project --------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(K1946VK035_BLDC_BOARD VERSION 1.0 LANGUAGES C ASM)
option(ENABLE_EXPERIMENTAL_PROTECTION "Enable unverified PWM TZ/HD protection" OFF)
set(CMAKE_C_STANDARD 11)

set(TARGET K1946VK035_BLDC_BOARD)
set(SRC_DIR K1921VK035_POWER_BOARD)

#-- Sources --------------------------------------------------------------------

add_executable(${TARGET}
    ${SRC_DIR}/src/main.c
    ${SRC_DIR}/src/PWM_Control.c
    ${SRC_DIR}/src/UART_Debug.c
    ${SRC_DIR}/src/FOC_Math.c
    ${SRC_DIR}/src/FOC_Core.c
    ${SRC_DIR}/src/FOC_Control.c
    ${SRC_DIR}/src/FOC_Protection.c
    ${SRC_DIR}/src/FOC_Observer.c
    ${SRC_DIR}/src/FOC_Svm.c
    ${SRC_DIR}/src/FOC_Params.c
    ${SRC_DIR}/src/FOC_Proto.c
    ${SRC_DIR}/src/FOC_Detect.c
    ${SRC_DIR}/src/VESC_Math.c
    ${SRC_DIR}/src/system_K1921VK035.c
    ${SRC_DIR}/asm/startup_K1921VK035.S
)

target_include_directories(${TARGET} PRIVATE ${SRC_DIR}/include)

#-- Compiler and linker --------------------------------------------------------

set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

target_compile_definitions(${TARGET} PRIVATE
    K1921VK035
    "__weak=__attribute__((weak))"
    "__packed=__attribute__((__packed__))"
    # startup вызывает `bl __START`; без newlib crt0 прыгаем сразу в main()
    __START=main
)
if(ENABLE_EXPERIMENTAL_PROTECTION)
    target_compile_definitions(${TARGET} PRIVATE ENABLE_EXPERIMENTAL_PROTECTION)
endif()

target_compile_options(${TARGET} PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections -fdata-sections -fsigned-char
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
    $<$<CONFIG:Release>:-Os>
    $<$<CONFIG:Debug>:-Og -g3>
)

# Без -fsingle-precision-constant литералы вроде 0.1963 становятся double
# и тянут софт-FP (__aeabi_dmul). Только для FOC/VESC исходников, чтобы не
set_source_files_properties(
    ${SRC_DIR}/src/FOC_Math.c
    ${SRC_DIR}/src/FOC_Core.c
    ${SRC_DIR}/src/FOC_Control.c
    ${SRC_DIR}/src/FOC_Observer.c
    ${SRC_DIR}/src/FOC_Svm.c
    ${SRC_DIR}/src/VESC_Math.c
    ${SRC_DIR}/src/UART_Debug.c
    PROPERTIES COMPILE_OPTIONS "-fsingle-precision-constant;-Wdouble-promotion"
)

target_link_options(${TARGET} PRIVATE
    ${MCU_FLAGS}
    -nostartfiles -specs=nano.specs -specs=nosys.specs
    -Wl,--entry=Reset_Handler
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET}.map
    -T ${CMAKE_CURRENT_LIST_DIR}/ld/K1921VK035.ld
)
target_link_libraries(${TARGET} PRIVATE m)

add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   ${TARGET}.elf ${TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET}.elf ${TARGET}.bin
    COMMAND ${CMAKE_SIZE} ${TARGET}.elf
)

#-- Host tests -----------------------------------------------------------------
set(_P ${CMAKE_CURRENT_LIST_DIR})
find_program(FOC_HOST_CC NAMES gcc cc clang NO_CMAKE_FIND_ROOT_PATH)
if(NOT FOC_HOST_CC)
    # Firmware-only builds do not require a native compiler.
    set(FOC_HOST_CC cc)
endif()
add_custom_target(host_tests
    COMMAND ${FOC_HOST_CC} -std=c11 -D_DEFAULT_SOURCE -DFOC_HOST_TEST -I${_P}/${SRC_DIR}/include -I${_P}/tests
            ${_P}/tests/test_foc.c
            ${_P}/${SRC_DIR}/src/FOC_Math.c ${_P}/${SRC_DIR}/src/FOC_Core.c
            ${_P}/${SRC_DIR}/src/FOC_Control.c ${_P}/${SRC_DIR}/src/FOC_Observer.c
            ${_P}/${SRC_DIR}/src/FOC_Svm.c ${_P}/${SRC_DIR}/src/VESC_Math.c
            ${_P}/${SRC_DIR}/src/FOC_Params.c ${_P}/${SRC_DIR}/src/FOC_Proto.c
            ${_P}/${SRC_DIR}/src/FOC_Detect.c ${_P}/${SRC_DIR}/src/FOC_Protection.c
            -lm -o ${CMAKE_BINARY_DIR}/test_foc
    COMMAND ${CMAKE_BINARY_DIR}/test_foc
    COMMAND ${FOC_HOST_CC} -std=c11 -I${_P}/${SRC_DIR}/include
            ${_P}/tests/test_protection.c ${_P}/${SRC_DIR}/src/FOC_Protection.c
            -o ${CMAKE_BINARY_DIR}/test_protection
    COMMAND ${CMAKE_BINARY_DIR}/test_protection
    VERBATIM
)

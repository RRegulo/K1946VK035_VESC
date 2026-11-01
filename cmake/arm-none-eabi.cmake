set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

find_program(_ARM_GCC arm-none-eabi-gcc)
if(NOT _ARM_GCC)
    message(FATAL_ERROR "arm-none-eabi-gcc not found in PATH")
endif()
get_filename_component(_ARM_BIN "${_ARM_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER   "${_ARM_GCC}")
set(CMAKE_ASM_COMPILER "${_ARM_GCC}")
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS "${_ARM_BIN}" REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size HINTS "${_ARM_BIN}" REQUIRED)

set(CMAKE_EXECUTABLE_SUFFIX_C   .elf)
set(CMAKE_EXECUTABLE_SUFFIX_ASM .elf)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

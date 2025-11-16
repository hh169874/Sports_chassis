cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME f401_05_3)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16 -O1)#O1-10ms O3-8ms
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
        Middlewares/ST/ARM/DSP/Lib
#        Core/Src/bsp/eMPL/lib
)

file(GLOB USER_SOURCES
        "Core/Src/bsp/*.c"
#        "Core/Src/bsp/eMPL/*.c"
        "Core/Src/bsp/VL53L1/core/*.c"
        "Core/Src/bsp/VL53L1/platform/*.c"
)
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        retarget/retarget.c
        ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        retarget/
        Core/Src/bsp/
#        Core/Src/bsp/eMPL
        Core/Src/bsp/VL53L1/core
        Core/Src/bsp/VL53L1/platform
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols

)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
        libarm_cortexM4lf_math.a
    # Add user defined libraries

)
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${CMAKE_PROJECT_NAME}.hex"
        COMMENT "Generating HEX file: ${CMAKE_PROJECT_NAME}.hex"
)

# 可选：生成 BIN 文件
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${CMAKE_PROJECT_NAME}.bin"
        COMMENT "Generating BIN file: ${CMAKE_PROJECT_NAME}.bin"
)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -specs=nosys.specs -u _printf_float -u _scanf_float")

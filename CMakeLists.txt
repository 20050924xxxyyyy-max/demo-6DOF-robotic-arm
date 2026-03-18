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
set(CMAKE_PROJECT_NAME sp_custom_controller_26)
set(CMSIS "${CMAKE_SOURCE_DIR}/Drivers/CMSIS")

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMSIS}/DSP/Lib/GCC
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    applications/controllers/joint_motor_controller/joint_motor_controller.cpp

    Matrix_and_Robotics_on_STM32/src/matrix.cpp
    Matrix_and_Robotics_on_STM32/src/robotics.cpp

    applications/can.cpp
    applications/can_recv.cpp
    applications/uart_task.cpp
    applications/buzzer_task.cpp
    applications/led_task.cpp
    applications/plot_task.cpp
    applications/send_task/send_task.cpp
    applications/control_task.cpp
    applications/controllers/joint_motor_controller/joint_motor_controller.cpp
    
    sp_middleware/io/fdcan/fdcan.cpp
    sp_middleware/io/led/led.cpp
    sp_middleware/io/buzzer/buzzer.cpp
    sp_middleware/io/ws2812/ws2812.cpp
    sp_middleware/io/plotter/plotter.cpp
    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/motor/dm_motor/dm_motor.cpp
    sp_middleware/motor/lk_motor/lk_motor.cpp
    
    sp_middleware/tools/math_tools/math_tools.cpp
    sp_middleware/tools/pid/pid.cpp
    sp_middleware/tools/mecanum/mecanum.cpp
    sp_middleware/tools/diff_gear/diff_gear.cpp
    sp_middleware/tools/motor_composer/motor_composer.cpp
    sp_middleware/tools/crc/crc.cpp
    sp_middleware/tools/low_pass_filter/low_pass_filter.cpp
    sp_middleware/referee/pm02/pm02.cpp 
    sp_middleware/referee/ui/element.cpp 
    sp_middleware/referee/ui/manager.cpp 
    sp_middleware/tools/crc/crc.cpp 
    sp_middleware/referee/vt03/vt03.cpp

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    applications
    sp_middleware
    Matrix_and_Robotics_on_STM32

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    __FPU_PRESENT=1U
    # Add user defined symbols

)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    arm_cortexM7lfsp_math
    # Add user defined libraries
)

# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/mnt/c/Users/Mark/Documents/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(four_motor_drive C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(four_motor_drive four_motor_drive.c)

pico_generate_pio_header(four_motor_drive ${CMAKE_CURRENT_LIST_DIR}/four_motor_drive.pio)

pico_set_program_name(four_motor_drive "four_motor_drive")
pico_set_program_version(four_motor_drive "0.1")

pico_enable_stdio_uart(four_motor_drive 1)
pico_enable_stdio_usb(four_motor_drive 1)

# Add the standard library to the build
target_link_libraries(four_motor_drive
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(four_motor_drive PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(four_motor_drive pico_stdlib pico_multicore hardware_pio hardware_timer hardware_pwm
        )

pico_add_extra_outputs(four_motor_drive)


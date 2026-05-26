# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# Point PICO_SDK_PATH at your local pico-sdk checkout, e.g.:
#   cmake -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pimoroni_pico_plus2 ..
# Or set the environment variable before invoking cmake.
# pico-vscode.cmake (added by the VS Code extension above) sets PICO_SDK_PATH
# as a CMake variable; fall back to the environment variable if building manually.
if(NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(icm42688_streamer C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# RP2350 board — Pimoroni Pico Plus 2
# Pass -DPICO_BOARD=pimoroni_pico_plus2 on the cmake command line,
# or uncomment the line below to hard-code it:
# set(PICO_BOARD pimoroni_pico_plus2)

pico_sdk_init()

add_executable(icm42688_streamer main.c)


target_link_libraries(icm42688_streamer
    pico_stdlib        # gpio, sleep, tight_loop_contents
    pico_multicore     # multicore_launch_core1
    hardware_spi       # spi_init, spi_write_read_blocking
    hardware_gpio      # gpio_set_irq_enabled_with_callback
)

# USB CDC for binary streaming; UART stdio disabled (no UART needed)
pico_enable_stdio_usb(icm42688_streamer  1)
pico_enable_stdio_uart(icm42688_streamer 0)

# Generates .uf2 (drag-and-drop flash) and .hex alongside the ELF
pico_add_extra_outputs(icm42688_streamer)

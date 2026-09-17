# Axon ROS Node Firmware
# Target: RoboCore Axon 2 Board, rev2 (RP2350A, 30 GPIO)
#
# Native Pico-ROS (zenoh) node: ST3215 + DDSM210 motor control in
# firmware, lidar UART forwarded over USB CDC.

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform type")

# rev2 board uses the RP2350A (30 GPIO, GP0..29). Match the SDK to the A part.
# (rev1 was the RP2354B/48-GPIO; PICO_RP2350A=0 selected that — gone on rev2.)
add_compile_definitions(PICO_RP2350A=1)

include(pico_sdk_import.cmake)

project(axon_firmware C CXX ASM)

pico_sdk_init()

# ---------------------------------------------------------------------------
# zenoh-pico — single-threaded bare-metal build with the custom "axon"
# platform profile (lib/zenoh-pico/cmake/platforms/axon.cmake). The only
# link is the zenoh serial protocol over our USB CDC port.
# ---------------------------------------------------------------------------
set(AXON_ZENOH_PLATFORM_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/zenoh_port/system_axon.c
    ${CMAKE_CURRENT_LIST_DIR}/src/zenoh_port/serial_axon.c
    CACHE INTERNAL "")
set(AXON_ZENOH_PLATFORM_INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}/src/zenoh_port
    ${CMAKE_CURRENT_LIST_DIR}            # tusb_config.h
    CACHE INTERNAL "")

set(ZP_PLATFORM "axon" CACHE STRING "" FORCE)
set(BUILD_SHARED_LIBS OFF)

set(Z_FEATURE_MULTI_THREAD 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_SERIAL 1 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_SERIAL_USB 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_TCP 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_UDP_MULTICAST 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_UDP_UNICAST 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_WS 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_TLS 0 CACHE STRING "" FORCE)
set(Z_FEATURE_SCOUTING 0 CACHE STRING "" FORCE)
set(Z_FEATURE_MULTICAST_TRANSPORT 0 CACHE STRING "" FORCE)
set(Z_FEATURE_RAWETH_TRANSPORT 0 CACHE STRING "" FORCE)
set(Z_FEATURE_UNICAST_PEER 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LIVELINESS 1 CACHE STRING "" FORCE)
set(Z_FEATURE_AUTO_RECONNECT 1 CACHE STRING "" FORCE)

# Match rmw_zenohd's default lease settings (pico-ros recommendation)
set(Z_TRANSPORT_LEASE 60000 CACHE STRING "" FORCE)
set(Z_TRANSPORT_LEASE_EXPIRE_FACTOR 2 CACHE STRING "" FORCE)

add_subdirectory(lib/zenoh-pico)

target_link_libraries(zenohpico_static
    pico_stdlib
    pico_rand
    tinyusb_device
    tinyusb_board
)

# ---------------------------------------------------------------------------
# Micro-CDR
# ---------------------------------------------------------------------------
add_library(microcdr STATIC
    lib/microcdr/src/c/common.c
    lib/microcdr/src/c/types/array.c
    lib/microcdr/src/c/types/basic.c
    lib/microcdr/src/c/types/sequence.c
    lib/microcdr/src/c/types/string.c
)
target_include_directories(microcdr PUBLIC
    lib/microcdr/include
    lib/microcdr/config
)

# ---------------------------------------------------------------------------
# Pico-ROS (picoros + picoserdes with our message types)
# ---------------------------------------------------------------------------
add_library(picoros STATIC
    lib/picoros/picoros.c
    lib/picoros/picoserdes.c
)
target_include_directories(picoros PUBLIC
    lib/picoros
    src/ros
)
target_compile_definitions(picoros PUBLIC -DUSER_TYPE_FILE="axon_types.h")
target_link_libraries(picoros zenohpico::lib microcdr)

# ---------------------------------------------------------------------------
# Firmware — native Pico-ROS node (rev2 / RP2350A)
# ---------------------------------------------------------------------------
add_executable(axon_firmware
    main.c
    usb_descriptors.c
    src/dbg.c
    src/bus/half_duplex.c
    src/bus/lidar_uart.c
    src/bus/ddsm_port.c
    src/motors/ddsm210.c
    src/motors/st3215.c
    src/sensors/bno055.c
    src/config/axon_cfg.c
    src/config/cfg_console.c
    src/config/motor_test.c
    src/ros/axon_node.c
    src/led.c
)

pico_set_program_name(axon_firmware "Axon ROS Node")
pico_set_program_version(axon_firmware "3.0")

# The firmware manages USB itself (custom 3-CDC TinyUSB descriptors); keep the
# SDK stdio off both transports so it cannot fight TinyUSB or claim a CDC.
pico_enable_stdio_uart(axon_firmware 0)
pico_enable_stdio_usb(axon_firmware 0)

target_include_directories(axon_firmware PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}        # pins.h, tusb_config.h, usb_descriptors.h
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(axon_firmware
    pico_stdlib
    pico_unique_id
    pico_rand
    hardware_pio
    hardware_clocks
    hardware_uart
    hardware_i2c
    hardware_irq
    hardware_flash
    hardware_watchdog
    tinyusb_device
    tinyusb_board
    picoros
)

pico_add_extra_outputs(axon_firmware)
pico_generate_pio_header(axon_firmware ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_tx.pio)
pico_generate_pio_header(axon_firmware ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_rx.pio)
pico_generate_pio_header(axon_firmware ${CMAKE_CURRENT_LIST_DIR}/src/pio/ws2812.pio)

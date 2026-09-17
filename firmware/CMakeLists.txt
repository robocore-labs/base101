# base101 firmware — a ROS 2 node on a Link101 board (RP2350A).
#
# The drivers, the board support and the whole zenoh/ROS stack are
# libraries now (link-libraries/, and easypicoros); what is left here is
# the robot. See robot.h.

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

# The board is an RP2350A: 30 GPIO, GP0..29. (The rev1 board was the
# 48-GPIO RP2354B, which is what PICO_RP2350A=0 selected.)
add_compile_definitions(PICO_RP2350A=1)

include(pico_sdk_import.cmake)

project(base101_firmware C CXX ASM)

pico_sdk_init()

# ---------------------------------------------------------------------------
# Libraries
#
# Five of these are the link101 libraries (git submodules under lib/); the
# sixth, easypicoros, brings Pico-ROS with it. Between them they are the
# board, the drivers, and the ROS stack:
#
#   pico_serial       the serial_t every driver speaks
#   pico_cdc_serial   a USB CDC interface as one of those
#   hardware_link101  the board: PIO/UART ports, pins, the LED strip, the
#                     onboard LSM6DSOX + MMC5983MA IMU
#   pico_feetech      Feetech STS servos
#   pico_ddsm         DDSM210 wheel motors
#   pico_zenoh        zenoh-pico for bare metal, over any serial_t
#   easypicoros       ROS 2 publishers and subscribers on top of that
#
# zenoh-pico, picoros and micro-CDR stay vendored in lib/: they are
# upstream projects we track rather than ours.
# ---------------------------------------------------------------------------
set(LINK101_LIBS ${CMAKE_CURRENT_LIST_DIR}/lib)

# tusb_config.h and the USB descriptors are ours, as TinyUSB requires.
set(CDC_SERIAL_TUSB_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "" FORCE)

# Where pico_zenoh and easypicoros find the upstream sources they build.
set(PICO_ZENOH_ZENOH_PICO_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/zenoh-pico)
set(EASYP_PICOROS_DIR         ${CMAKE_CURRENT_LIST_DIR}/lib/picoros)
set(EASYP_MICROCDR_DIR        ${CMAKE_CURRENT_LIST_DIR}/lib/microcdr)

add_subdirectory(${LINK101_LIBS}/pico_serial      pico_serial)
add_subdirectory(${LINK101_LIBS}/pico_cdc_serial  pico_cdc_serial)
add_subdirectory(${LINK101_LIBS}/pico_zenoh       pico_zenoh)
add_subdirectory(${LINK101_LIBS}/hardware_link101 hardware_link101)
add_subdirectory(${LINK101_LIBS}/pico_feetech     pico_feetech)
add_subdirectory(${LINK101_LIBS}/pico_ddsm        pico_ddsm)
add_subdirectory(${LINK101_LIBS}/easypicoros      easypicoros)

# ---------------------------------------------------------------------------
# The firmware
# ---------------------------------------------------------------------------
add_executable(base101_firmware
    main.c
    io.c
    lidar.c
    wheels.c
    servos.c
    imu.c
    ros.c
    status.c
    usb_descriptors.c
)

pico_set_program_name(base101_firmware "base101")
pico_set_program_version(base101_firmware "4.0")

# The firmware drives USB itself (three CDC ports, custom descriptors), so
# the SDK's stdio must stay off both transports and out of the way.
pico_enable_stdio_uart(base101_firmware 0)
pico_enable_stdio_usb(base101_firmware 0)

target_include_directories(base101_firmware PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(base101_firmware
    pico_stdlib
    pico_unique_id
    hardware_i2c
    tinyusb_device
    tinyusb_board
    hardware_link101
    pico_cdc_serial
    pico_feetech
    pico_ddsm
    easypicoros
)

target_compile_options(base101_firmware PRIVATE -Wall -Wextra)

pico_add_extra_outputs(base101_firmware)

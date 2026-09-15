# Axon (RP2350, bare metal) platform profile — added by axon-firmware.
#
# The system layer and serial link live in the firmware tree
# (src/zenoh_port). The top-level CMakeLists passes their paths in via
# the AXON_ZENOH_PLATFORM_SOURCES / AXON_ZENOH_PLATFORM_INCLUDES cache
# variables before add_subdirectory(lib/zenoh-pico).
set(ZP_PLATFORM_SYSTEM_LAYER axon)
set(ZP_PLATFORM_COMPILE_DEFINITIONS ZENOH_GENERIC)
set(ZP_PLATFORM_SOURCE_FILES ${AXON_ZENOH_PLATFORM_SOURCES})
set(ZP_PLATFORM_INCLUDE_DIRS ${AXON_ZENOH_PLATFORM_INCLUDES})
set(CHECK_THREADS OFF)

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(beacon_1)

# Include directories for nRF Connect SDK components
# The previous attempt might have failed because we need to link against the specific library or the path structure is slightly different.
# Let's try to add the specific path where the service headers reside.
# Based on common NCS structure: nrf/include/bluetooth/services

get_filename_component(NCS_ROOT "${ZEPHYR_BASE}/../nrf" ABSOLUTE)
target_include_directories(app PRIVATE ${NCS_ROOT}/include)

target_sources(app PRIVATE src/main.c)

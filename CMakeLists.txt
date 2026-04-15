cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

add_subdirectory(drivers/bno086)
add_subdirectory(drivers/ads1299)

project(snoozy_final)

set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(app PRIVATE
    src/main.c
)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(snoozy_final)

target_sources(app PRIVATE
    src/main.c
    src/ble.c
    src/ads.c
    src/imu.c
    src/power.c
)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
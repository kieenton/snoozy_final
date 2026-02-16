cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(snoozy_final)

target_sources(app PRIVATE
    src/main.c
    src/ble.c
)


add_subdirectory(drivers/ads1299)
add_subdirectory(drivers/bno086)
zephyr_library()

zephyr_library_sources(
    bno086.c
    bno086_hal.c
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)

zephyr_include_directories(.)
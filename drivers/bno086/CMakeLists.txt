if(CONFIG_BNO086)

zephyr_library()

add_dependencies(${ZEPHYR_CURRENT_LIBRARY} offsets_h)

zephyr_library_sources(
    bno086.c
    bno086_hal.c
    sh2.c
    sh2_SensorValue.c
    sh2_util.c
    shtp.c
)

target_link_libraries(app PRIVATE ${ZEPHYR_CURRENT_LIBRARY})

endif()
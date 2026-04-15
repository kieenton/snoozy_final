zephyr_library()

add_dependencies(${ZEPHYR_CURRENT_LIBRARY} offsets_h)

zephyr_library_sources(ads1299.c)
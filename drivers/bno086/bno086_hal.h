#ifndef BNO086_HAL_H
#define BNO086_HAL_H

#include <stdbool.h>
#include "sh2_hal.h"

/**
 * Returns the sh2_Hal_t instance wired to Zephyr's SPI/GPIO.
 * Pass this pointer to sh2_open().
 */
sh2_Hal_t *bno086_hal_get(void);

/**
 * Returns true if the INT pin fired since the last read().
 * Useful for polling loops that want to avoid unnecessary sh2_service() calls.
 */
bool bno086_hal_int_asserted(void);

#endif /* BNO086_HAL_H */
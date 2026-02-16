#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>

void ble_init(void);
bool ble_is_connected(void);
int ble_send(const uint8_t *data, uint16_t len);

#endif

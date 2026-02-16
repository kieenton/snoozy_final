#include "ble.h"
#include <zephyr/kernel.h>

int main(void)
{
    ble_init();

    while (1) {

        if (ble_is_connected()) {
            char msg[] = "Hello from Snoozy\r\n";
            ble_send(msg, sizeof(msg) - 1);
        }

        k_sleep(K_SECONDS(1));
    }
}
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/devicetree.h>

const struct device *gpio2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));

int main(void)
{
    // Temporarily claim MOSI pin as GPIO output
    gpio_pin_configure(gpio2, 4, GPIO_OUTPUT);  // P2.04 = MOSI
    while(1){
        gpio_pin_set(gpio2, 4, 1);
        k_sleep(K_MSEC(100));
        gpio_pin_set(gpio2, 4, 0);
        k_sleep(K_MSEC(100));
    }
}
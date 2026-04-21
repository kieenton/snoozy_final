#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(gpio_test, LOG_LEVEL_DBG);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/* Control pins */
static const struct gpio_dt_spec rst = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_reset_gpios);
static const struct gpio_dt_spec start = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_start_gpios);
static const struct gpio_dt_spec drdy = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_drdy_gpios);
static const struct gpio_dt_spec pwdwn = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_pwdwn_gpios);
static const struct gpio_dt_spec cs    = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi20), cs_gpios, 0);

int main(void)
{
    k_sleep(K_MSEC(5000));
    LOG_INF("=== GPIO Toggle Test ===");

    /* Verify all GPIO controllers are ready before use */
    if (!device_is_ready(pwdwn.port)) { LOG_ERR("pwdwn port not ready"); return -1; }
    if (!device_is_ready(rst.port))   { LOG_ERR("rst port not ready");   return -1; }
    if (!device_is_ready(start.port)) { LOG_ERR("start port not ready"); return -1; }
    if (!device_is_ready(cs.port))    { LOG_ERR("cs port not ready");    return -1; }
    if (!device_is_ready(drdy.port))  { LOG_ERR("drdy port not ready");  return -1; }

    /* Configure outputs — GPIO_OUTPUT_ACTIVE drives pin to its active state.
     * All pins are GPIO_ACTIVE_LOW in DT, so ACTIVE = physical LOW.
     * Use GPIO_OUTPUT_INACTIVE to start deasserted (physical HIGH). */
    gpio_pin_configure_dt(&pwdwn, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&rst,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&start, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&cs,    GPIO_OUTPUT_INACTIVE);

    /* Configure input */
    gpio_pin_configure_dt(&drdy, GPIO_INPUT);

    LOG_INF("All pins configured");

    while (1) {
        /* NOTE: pins are GPIO_ACTIVE_LOW — set(1) = physical LOW, set(0) = physical HIGH */
        LOG_INF("PWDN: physical LOW  (P1.06)"); gpio_pin_set_dt(&pwdwn, 1); k_sleep(K_MSEC(500));
        LOG_INF("PWDN: physical HIGH (P1.06)"); gpio_pin_set_dt(&pwdwn, 0); k_sleep(K_MSEC(500));

        LOG_INF("RST:  physical LOW  (P1.05)"); gpio_pin_set_dt(&rst,   1); k_sleep(K_MSEC(500));
        LOG_INF("RST:  physical HIGH (P1.05)"); gpio_pin_set_dt(&rst,   0); k_sleep(K_MSEC(500));

        LOG_INF("START: physical LOW  (P1.04)"); gpio_pin_set_dt(&start, 1); k_sleep(K_MSEC(500));
        LOG_INF("START: physical HIGH (P1.04)"); gpio_pin_set_dt(&start, 0); k_sleep(K_MSEC(500));

        LOG_INF("CS: physical LOW  (P2.05)"); gpio_pin_set_dt(&cs, 1); k_sleep(K_MSEC(500));
        LOG_INF("CS: physical HIGH (P2.05)"); gpio_pin_set_dt(&cs, 0); k_sleep(K_MSEC(500));

        LOG_INF("DRDY = %d (P0.01)", gpio_pin_get_dt(&drdy));

        LOG_INF("--- next cycle ---");
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
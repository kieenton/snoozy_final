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
    LOG_INF("=== GPIO Toggle Test ===");

    /* Configure outputs */
    gpio_pin_configure_dt(&pwdwn, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&rst,   GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&start, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&cs,    GPIO_OUTPUT_INACTIVE);

    /* Configure input */
    gpio_pin_configure_dt(&drdy, GPIO_INPUT);

    LOG_INF("All pins configured");

    while (1) {
        LOG_INF("PWDN HIGH (P1.06)"); gpio_pin_set_dt(&pwdwn, 1); k_sleep(K_MSEC(500));
        LOG_INF("PWDN LOW");          gpio_pin_set_dt(&pwdwn, 0); k_sleep(K_MSEC(500));

        LOG_INF("RST HIGH  (P1.05)"); gpio_pin_set_dt(&rst,   1); k_sleep(K_MSEC(500));
        LOG_INF("RST LOW");           gpio_pin_set_dt(&rst,   0); k_sleep(K_MSEC(500));

        LOG_INF("START HIGH (P1.04)"); gpio_pin_set_dt(&start, 1); k_sleep(K_MSEC(500));
        LOG_INF("START LOW");          gpio_pin_set_dt(&start, 0); k_sleep(K_MSEC(500));

        LOG_INF("CS HIGH  (P2.05)");  gpio_pin_set_dt(&cs,    1); k_sleep(K_MSEC(500));
        LOG_INF("CS LOW");            gpio_pin_set_dt(&cs,    0); k_sleep(K_MSEC(500));

        LOG_INF("DRDY = %d (P0.01)", gpio_pin_get_dt(&drdy));

        LOG_INF("--- next cycle ---");
        k_sleep(K_MSEC(1000));
    }

    return 0;
}
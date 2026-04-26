#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(power_mgr, LOG_LEVEL_INF);

/* nPM1304 I2C address */
#define NPM1304_ADDR        0x6B

/* Ship mode register (same as nPM1300) */
#define NPM1304_SHIP_BASE   0x0B
#define NPM1304_SHIP_OFFSET 0x01  /* TASKSHIPMODE register */
#define NPM1304_SHIP_VAL    0x01  /* Trigger ship mode task */

#define I2C_NODE DT_NODELABEL(i2c21)
static const struct device *i2c = DEVICE_DT_GET(I2C_NODE);

static int enter_ship_mode(void)
{
    /* Write 0x01 to TASKSHIPMODE register (base 0x0B, offset 0x01)
     * nPM1304 uses 16-bit register addressing: [base_addr, offset, value] */
    uint8_t buf[] = { NPM1304_SHIP_BASE, NPM1304_SHIP_OFFSET, NPM1304_SHIP_VAL };
    return i2c_write(i2c, buf, sizeof(buf), NPM1304_ADDR);
}

int main(void)
{
    if (!device_is_ready(i2c)) {
        LOG_ERR("I2C not ready");
        return -ENODEV;
    }

    LOG_INF("System on — LED should be lit");

    /* Wait 5 seconds so you can see the LED is on */
    k_msleep(5000);

    LOG_INF("Entering ship mode — LED should turn off");
    k_msleep(50);

    int ret = enter_ship_mode();
    if (ret != 0) {
        LOG_ERR("Failed to enter ship mode: %d", ret);
        return ret;
    }

    /* Should never reach here */
    return 0;
}
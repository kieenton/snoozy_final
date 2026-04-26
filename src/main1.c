#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#define CMD_SDATAC 0x11
#define CMD_RREG   0x20
#define REG_ID     0x00

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct spi_dt_spec ads_spi =
    SPI_DT_SPEC_GET(DT_NODELABEL(ads1299),
                    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
                    0);

static const struct gpio_dt_spec reset_pin =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_reset_gpios);

static const struct gpio_dt_spec start_pin =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_start_gpios);

static const struct gpio_dt_spec pwdwn_pin =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_pwdwn_gpios);

static int read_id_register(void)
{
    int ret;

    uint8_t tx_cmd[] = { CMD_SDATAC };
    struct spi_buf txb1 = {
        .buf = tx_cmd,
        .len = sizeof(tx_cmd),
    };
    struct spi_buf_set txs1 = {
        .buffers = &txb1,
        .count = 1,
    };

    ret = spi_write_dt(&ads_spi, &txs1);
    if (ret) {
        printk("spi_write_dt(SDATAC) failed: %d\n", ret);
        return ret;
    }

    k_sleep(K_MSEC(10));

    uint8_t tx_read[] = { CMD_RREG | REG_ID, 0x00, 0x00 };
    uint8_t rx_read[3] = {0};

    struct spi_buf txb2 = {
        .buf = tx_read,
        .len = sizeof(tx_read),
    };
    struct spi_buf_set txs2 = {
        .buffers = &txb2,
        .count = 1,
    };

    struct spi_buf rxb2 = {
        .buf = rx_read,
        .len = sizeof(rx_read),
    };
    struct spi_buf_set rxs2 = {
        .buffers = &rxb2,
        .count = 1,
    };

    ret = spi_transceive_dt(&ads_spi, &txs2, &rxs2);
    if (ret) {
        printk("spi_transceive_dt(RREG) failed: %d\n", ret);
        return ret;
    }

    printk("RX bytes: %02X %02X %02X\n", rx_read[0], rx_read[1], rx_read[2]);
    printk("ADS1299 ID Register returned: 0x%02X\n", rx_read[2]);

    if (rx_read[2] == 0x3C) {
        printk("SUCCESS! Pure C Zephyr SPI is working.\n");
    } else {
        printk("ERROR: Expected 0x3C. Check wiring.\n");
    }

    return 0;
}

int main(void)
{
    int ret;

    k_sleep(K_MSEC(5000));
    printk("Starting Zephyr ADS1299 Test...\n");

    if (!spi_is_ready_dt(&ads_spi)) {
        printk("Fatal: SPI bus not ready!\n");
        return 0;
    }

    if (!gpio_is_ready_dt(&reset_pin) || !gpio_is_ready_dt(&start_pin)) {
        printk("Fatal: GPIO device not ready!\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&reset_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("reset configure failed: %d\n", ret);
        return 0;
    }

    ret = gpio_pin_configure_dt(&start_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("start configure failed: %d\n", ret);
        return 0;
    }

    ret = gpio_pin_configure_dt(&pwdwn_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("pwdwn configure failed: %d\n", ret);
        return 0;
    }
    
    gpio_pin_set_dt(&pwdwn_pin, 0);  // bring chip out of power down
    gpio_pin_set_dt(&reset_pin, 0);  // bring reset high
    k_sleep(K_MSEC(50));
    printk("OK: PWDN high\n");
    /* For active-low reset: logical 1 asserts reset low, logical 0 releases it high */
    gpio_pin_set_dt(&reset_pin, 1);
    k_sleep(K_MSEC(10));
    gpio_pin_set_dt(&reset_pin, 0);
    k_sleep(K_MSEC(50));

    read_id_register();
    return 0;
}
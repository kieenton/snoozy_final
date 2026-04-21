/*
 * bno086_hal.c
 *
 * Zephyr SPI transport layer for the Ceva sh2 library.
 * Implements the sh2_Hal_t interface defined in sh2_hal.h.
 *
 * Hardware connections (from your overlay):
 *   CS   -> gpio2 5  (index 1 in spi00 cs-gpios)
 *   INT  -> gpio1 3
 *   RST  -> gpio1 2
 *   BOOT -> gpio1 7  (not driven here, tie high for SPI mode)
 */

#include "bno086_hal.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

#include "sh2_hal.h"

/* ------------------------------------------------------------------ */
/* Devicetree bindings                                                  */
/* ------------------------------------------------------------------ */

#define BNO_NODE DT_NODELABEL(bno086)

static const struct spi_dt_spec bno_spi =
    SPI_DT_SPEC_GET(BNO_NODE,
                    SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                    SPI_MODE_CPOL | SPI_MODE_CPHA,   /* SPI mode 3 */
                    0);

static const struct gpio_dt_spec int_pin =
    GPIO_DT_SPEC_GET(BNO_NODE, int_gpios);

static const struct gpio_dt_spec rst_pin =
    GPIO_DT_SPEC_GET(BNO_NODE, rst_gpios);

static const struct gpio_dt_spec boot_pin =
    GPIO_DT_SPEC_GET(BNO_NODE, boot_gpios);

/* ------------------------------------------------------------------ */
/* Internal state                                                       */
/* ------------------------------------------------------------------ */

static struct gpio_callback int_cb_data;
static volatile bool        int_asserted;

/* ------------------------------------------------------------------ */
/* INT GPIO interrupt handler                                           */
/* ------------------------------------------------------------------ */

static void int_gpio_handler(const struct device *dev,
                              struct gpio_callback *cb,
                              uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int_asserted = true;
}

/* ------------------------------------------------------------------ */
/* sh2_Hal_t callbacks                                                  */
/* ------------------------------------------------------------------ */

/*
 * open() — called once by sh2_open().
 * Configure GPIOs, perform hardware reset, wait for HINT to assert.
 */
static int hal_open(sh2_Hal_t *self)
{
    ARG_UNUSED(self);
    int ret;

    /* BOOT high = normal sensor hub mode (not DFU bootloader) */
    ret = gpio_pin_configure_dt(&boot_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("BNO086: boot configure failed: %d\n", ret);
        return ret;
    }

    /* Configure RST as output, assert reset */
    ret = gpio_pin_configure_dt(&rst_pin, GPIO_OUTPUT_ACTIVE);
    if (ret) {
        printk("BNO086: rst configure failed: %d\n", ret);
        return ret;
    }

    /* Configure INT as input with interrupt on falling edge */
    ret = gpio_pin_configure_dt(&int_pin, GPIO_INPUT);
    if (ret) {
        printk("BNO086: int configure failed: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&int_pin, GPIO_INT_EDGE_FALLING);
    if (ret) {
        printk("BNO086: int interrupt configure failed: %d\n", ret);
        return ret;
    }

    gpio_init_callback(&int_cb_data, int_gpio_handler, BIT(int_pin.pin));
    gpio_add_callback(int_pin.port, &int_cb_data);

    /* Reset pulse: hold reset low for 10 ms, then release */
    gpio_pin_set_dt(&rst_pin, 1);   /* assert reset (active low) */
    k_sleep(K_MSEC(10));
    gpio_pin_set_dt(&rst_pin, 0);   /* deassert reset */
    k_sleep(K_MSEC(50));            /* wait for boot */

    int_asserted = false;
    printk("BNO086: HAL open\n");
    return 0;
}

/*
 * close() — called by sh2_close().
 */
static void hal_close(sh2_Hal_t *self)
{
    ARG_UNUSED(self);

    gpio_pin_interrupt_configure_dt(&int_pin, GPIO_INT_DISABLE);
    gpio_remove_callback(int_pin.port, &int_cb_data);
    printk("BNO086: HAL close\n");
}

/*
 * read() — called by shtp to receive data from BNO086.
 *
 * Returns number of bytes read, or 0 if HINT not asserted / no data.
 * The sh2 library calls this repeatedly from sh2_service().
 */
static int hal_read(sh2_Hal_t *self,
                    uint8_t   *pBuffer,
                    unsigned   len,
                    uint32_t  *t_us)
{
    ARG_UNUSED(self);

    /* Only read when HINT (INT pin) is asserted low */
    int hint = gpio_pin_get_dt(&int_pin);
    if (hint <= 0) {
        return 0;
    }

    /* Timestamp in microseconds */
    *t_us = (uint32_t)(k_uptime_get() * 1000U);

    /*
     * SHTP SPI protocol:
     *   First transaction: read 4-byte header to get payload length.
     *   Second transaction: read full frame (header + payload).
     *
     * We do it in one transfer up to len bytes; the sh2 library
     * passes a buffer large enough for SH2_HAL_MAX_TRANSFER_IN (1024).
     */
    uint8_t tx[SH2_HAL_MAX_TRANSFER_IN];
    memset(tx, 0, len);

    struct spi_buf     rx_buf  = { .buf = pBuffer, .len = len };
    struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

    struct spi_buf     tx_buf  = { .buf = tx, .len = len };
    struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

    int ret = spi_transceive_dt(&bno_spi, &tx_bufs, &rx_bufs);
    if (ret) {
        printk("BNO086: SPI read error %d\n", ret);
        return 0;
    }

    /* Length is encoded in the first two bytes of the SHTP header */
    uint16_t payload_len = ((uint16_t)pBuffer[0]) |
                           ((uint16_t)(pBuffer[1] & 0x7F) << 8);

    if (payload_len == 0) {
        return 0;
    }

    int_asserted = false;
    return (int)payload_len;
}

/*
 * write() — called by shtp to send data to BNO086.
 *
 * Returns number of bytes written, or 0 if busy.
 */
static int hal_write(sh2_Hal_t   *self,
                     uint8_t     *pBuffer,
                     unsigned     len)
{
    ARG_UNUSED(self);

    struct spi_buf     tx_buf  = { .buf = pBuffer, .len = len };
    struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

    int ret = spi_write_dt(&bno_spi, &tx_bufs);
    if (ret) {
        printk("BNO086: SPI write error %d\n", ret);
        return 0;
    }

    return (int)len;
}

/*
 * getTimeUs() — monotonic clock in microseconds.
 */
static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    ARG_UNUSED(self);
    return (uint32_t)(k_uptime_get() * 1000U);
}

/* ------------------------------------------------------------------ */
/* Public: HAL instance                                                 */
/* ------------------------------------------------------------------ */

static sh2_Hal_t bno086_hal = {
    .open       = hal_open,
    .close      = hal_close,
    .read       = hal_read,
    .write      = hal_write,
    .getTimeUs  = hal_get_time_us,
};

sh2_Hal_t *bno086_hal_get(void)
{
    return &bno086_hal;
}

bool bno086_hal_int_asserted(void)
{
    return int_asserted;
}
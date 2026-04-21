#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_test, LOG_LEVEL_DBG);

#include <bluetooth/services/nus.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CMD_WAKEUP  0x02
#define CMD_STANDBY 0x04
#define CMD_RESET   0x06
#define CMD_START   0x08
#define CMD_STOP    0x0A
#define CMD_RDATAC  0x10
#define CMD_SDATAC  0x11
#define CMD_RDATA   0x12
#define CMD_RREG    0x20
#define CMD_WREG    0x40

#define REG_ID         0x00
#define REG_CONFIG1    0x01
#define REG_CONFIG2    0x02
#define REG_CONFIG3    0x03
#define REG_LOFF       0x04
#define REG_CH1SET     0x05
#define REG_CH2SET     0x06
#define REG_CH3SET     0x07
#define REG_CH4SET     0x08
#define REG_CH5SET     0x09
#define REG_CH6SET     0x0A
#define REG_CH7SET     0x0B
#define REG_CH8SET     0x0C
#define REG_BIAS_SENSP 0x0D
#define REG_BIAS_SENSN 0x0E
#define REG_LOFF_SENSP 0x0F
#define REG_LOFF_SENSN 0x10
#define REG_LOFF_FLIP  0x11
#define REG_LOFF_STATP 0x12
#define REG_LOFF_STATN 0x13
#define REG_GPIO       0x14
#define REG_MISC1      0x15
#define REG_MISC2      0x16
#define REG_CONFIG4    0x17

#define ADS_NUM_CHANNELS 8
#define ADS_FRAME_BYTES (3 + ADS_NUM_CHANNELS * 3)

#define STREAM_DECIMATION 1U
#define PACKET_SIZE 40

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
#define DEVICE_NAME      CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN  (sizeof(DEVICE_NAME) - 1)

static const struct spi_dt_spec ads_spi =
    SPI_DT_SPEC_GET(DT_NODELABEL(ads1299),
                    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
                    0);

static const struct gpio_dt_spec reset_pin =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_reset_gpios);

static const struct gpio_dt_spec start_pin =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_start_gpios);

static const struct gpio_dt_spec drdy_pin =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_drdy_gpios);

static const struct gpio_dt_spec pwdwn_pin =
    GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ads_pwdwn_gpios);

static struct bt_conn *current_conn;
static bool ble_ready;
static bool stream_enabled;
static bool stream_requested;
static int64_t stream_start_at_ms;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL)
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static int ads_spi_write_bytes(const uint8_t *data, size_t len)
{
    struct spi_buf txb = {
        .buf = (void *)data,
        .len = len,
    };
    struct spi_buf_set txs = {
        .buffers = &txb,
        .count = 1,
    };

    return spi_write_dt(&ads_spi, &txs);
}

static int ads_spi_transceive_bytes(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    struct spi_buf txb = {
        .buf = (void *)tx_data,
        .len = len,
    };
    struct spi_buf_set txs = {
        .buffers = &txb,
        .count = 1,
    };

    struct spi_buf rxb = {
        .buf = rx_data,
        .len = len,
    };
    struct spi_buf_set rxs = {
        .buffers = &rxb,
        .count = 1,
    };

    return spi_transceive_dt(&ads_spi, &txs, &rxs);
}

static int ads_send_cmd(uint8_t cmd)
{
    int ret = ads_spi_write_bytes(&cmd, 1);
    k_busy_wait(10);
    return ret;
}

static int ads_read_reg(uint8_t reg, uint8_t *value)
{
    int ret;
    uint8_t tx[3] = {
        (uint8_t)(CMD_RREG | reg),
        0x00,
        0x00
    };
    uint8_t rx[3] = {0};

    ret = ads_spi_transceive_bytes(tx, rx, sizeof(tx));
    if (ret == 0) {
        *value = rx[2];
    }

    return ret;
}

static int ads_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[3] = {
        (uint8_t)(CMD_WREG | reg),
        0x00,
        value
    };

    return ads_spi_write_bytes(tx, sizeof(tx));
}

static int ads_write_regs(uint8_t start_reg, const uint8_t *values, size_t count)
{
    uint8_t tx[2 + 27];

    if (count == 0 || count > 27) {
        return -EINVAL;
    }

    tx[0] = (uint8_t)(CMD_WREG | start_reg);
    tx[1] = (uint8_t)(count - 1);

    for (size_t i = 0; i < count; i++) {
        tx[2 + i] = values[i];
    }

    return ads_spi_write_bytes(tx, 2 + count);
}

static int ads_wait_drdy_low_timeout_ms(int timeout_ms)
{
    int loops = timeout_ms * 10;

    for (int i = 0; i < loops; i++) {
        int val = gpio_pin_get_dt(&drdy_pin);
        if (val < 0) {
            return val;
        }

        if (val == 0) {
            return 0;
        }

        k_busy_wait(100);
    }

    return -ETIMEDOUT;
}

static int32_t ads_decode24(const uint8_t *p)
{
    int32_t v = ((int32_t)p[0] << 16) |
                ((int32_t)p[1] << 8)  |
                ((int32_t)p[2]);

    if (v & 0x800000) {
        v |= (int32_t)0xFF000000;
    }

    return v;
}

static int ads_read_frame_rdata(uint8_t frame[ADS_FRAME_BYTES])
{
    int ret;
    uint8_t tx[1 + ADS_FRAME_BYTES];
    uint8_t rx[1 + ADS_FRAME_BYTES];

    tx[0] = CMD_RDATA;
    for (int i = 1; i < (int)sizeof(tx); i++) {
        tx[i] = 0x00;
    }

    ret = ads_spi_transceive_bytes(tx, rx, sizeof(tx));
    if (ret) {
        return ret;
    }

    for (int i = 0; i < ADS_FRAME_BYTES; i++) {
        frame[i] = rx[i + 1];
    }

    return 0;
}

static int ads_dump_key_regs(void)
{
    static const uint8_t regs[] = {
        REG_ID, REG_CONFIG1, REG_CONFIG2, REG_CONFIG3,
        REG_CH1SET, REG_CH2SET, REG_CH3SET, REG_CH4SET,
        REG_CH5SET, REG_CH6SET, REG_CH7SET, REG_CH8SET,
        REG_BIAS_SENSP, REG_BIAS_SENSN, REG_MISC1, REG_MISC2, REG_CONFIG4
    };

    for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
        uint8_t v = 0;
        int ret = ads_read_reg(regs[i], &v);
        if (ret) {
            printk("reg 0x%02X read failed: %d\n", regs[i], ret);
            return ret;
        }
        printk("REG 0x%02X = 0x%02X\n", regs[i], v);
    }

    return 0;
}

static int ads_print_id(void)
{
    int ret;
    uint8_t id = 0;

    ret = ads_send_cmd(CMD_SDATAC);
    if (ret) return ret;
    k_sleep(K_MSEC(2));

    ret = ads_read_reg(REG_ID, &id);
    if (ret) return ret;

    printk("ADS1299 ID: 0x%02X\n", id);
    return 0;
}

static int ads_configure_external_ch1_mode(void)
{
    int ret;

    /*
     * CH1 = normal electrode input, gain 24, SRB2 open, powered on
     * CH2-CH8 = powered down
     *
     * 0x60 = 0110 0000
     *   PDn=0 (on)
     *   GAIN=110 (x24)
     *   MUX=000 (normal electrode input)
     *
     * 0xE1 = 1110 0001
     *   PDn=1 (off)
     *   GAIN=110 (x24, irrelevant when off)
     *   MUX=001
     */
    uint8_t chset_all[8] = {
        0x60, /* CH1 active, normal input */
        0xE1, /* CH2 off */
        0xE1, /* CH3 off */
        0xE1, /* CH4 off */
        0xE1, /* CH5 off */
        0xE1, /* CH6 off */
        0xE1, /* CH7 off */
        0xE1  /* CH8 off */
    };

    ret = ads_send_cmd(CMD_SDATAC);
    if (ret) return ret;
    k_sleep(K_MSEC(2));

    ret = ads_send_cmd(CMD_STOP);
    if (ret) return ret;
    k_sleep(K_MSEC(2));

    /* 250 SPS, daisy disabled */
    ret = ads_write_reg(REG_CONFIG1, 0x96);
    if (ret) return ret;

    /*
     * External signal mode:
     * internal test source disabled
     */
    ret = ads_write_reg(REG_CONFIG2, 0xC0);
    if (ret) return ret;

    /*
     * Keep internal reference buffer enabled, bias buffer on.
     * This stays close to your previous working setup.
     */
    ret = ads_write_reg(REG_CONFIG3, 0x60);
    if (ret) return ret;

    /* Disable lead-off for this simple bench test */
    ret = ads_write_reg(REG_LOFF, 0x00);
    if (ret) return ret;
    ret = ads_write_reg(REG_LOFF_SENSP, 0x00);
    if (ret) return ret;
    ret = ads_write_reg(REG_LOFF_SENSN, 0x00);
    if (ret) return ret;
    ret = ads_write_reg(REG_LOFF_FLIP, 0x00);
    if (ret) return ret;

    /*
     * No channels added to bias drive for this first external bench test.
     * Keep it simple while injecting a small external signal.
     */
    ret = ads_write_reg(REG_BIAS_SENSP, 0x00);
    if (ret) return ret;
    ret = ads_write_reg(REG_BIAS_SENSN, 0x00);
    if (ret) return ret;

    /*
     * SRB1 off, SRB2 handled per-channel via CHnSET.
     * CH1SET above keeps SRB2 disconnected.
     */
    ret = ads_write_reg(REG_MISC1, 0x00);
    if (ret) return ret;
    ret = ads_write_reg(REG_MISC2, 0x00);
    if (ret) return ret;
    ret = ads_write_reg(REG_CONFIG4, 0x00);
    if (ret) return ret;

    ret = ads_write_regs(REG_CH1SET, chset_all, sizeof(chset_all));
    if (ret) return ret;

    k_sleep(K_MSEC(2));
    return 0;
}

static void build_binary_packet(uint8_t pkt[PACKET_SIZE], uint32_t sample_idx,
                                const uint8_t frame[ADS_FRAME_BYTES])
{
    uint32_t status = ((uint32_t)frame[0] << 16) |
                      ((uint32_t)frame[1] << 8)  |
                      ((uint32_t)frame[2]);

    sys_put_le32(sample_idx, &pkt[0]);
    sys_put_le32(status, &pkt[4]);

    for (int ch = 0; ch < ADS_NUM_CHANNELS; ch++) {
        int32_t sample = ads_decode24(&frame[3 + ch * 3]);
        sys_put_le32((uint32_t)sample, &pkt[8 + ch * 4]);
    }
}

static void nus_send_binary(const uint8_t *data, uint16_t len)
{
    if (current_conn) {
        int err = bt_nus_send(current_conn, data, len);
        if (err) {
            printk("bt_nus_send err=%d\n", err);
        }
    }
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
              uint16_t len)
{
    char msg[64];
    size_t n = MIN((size_t)len, sizeof(msg) - 1);

    memcpy(msg, data, n);
    msg[n] = '\0';

    printk("BLE RX: %s\n", msg);

    if (strstr(msg, "start") != NULL) {
        stream_requested = true;
        stream_enabled = false;
        stream_start_at_ms = k_uptime_get() + 200;
        printk("Streaming requested\n");
    } else if (strstr(msg, "stop") != NULL) {
        stream_requested = false;
        stream_enabled = false;
        printk("Streaming disabled\n");
    }
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE connect failed: %u\n", err);
        return;
    }

    if (current_conn != NULL) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    current_conn = bt_conn_ref(conn);
    stream_requested = false;
    stream_enabled = false;
    printk("BLE connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);

    if (current_conn != NULL) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    stream_requested = false;
    stream_enabled = false;
    printk("BLE disconnected\n");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static int ble_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("bt_enable failed: %d\n", err);
        return err;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        printk("bt_nus_init failed: %d\n", err);
        return err;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
                          ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        printk("bt_le_adv_start failed: %d\n", err);
        return err;
    }

    ble_ready = true;
    printk("BLE advertising as %s\n", DEVICE_NAME);
    return 0;
}

int main(void)
{
    int ret;
    uint8_t frame[ADS_FRAME_BYTES];
    uint8_t pkt[PACKET_SIZE];
    uint32_t sample_idx = 0;

    k_sleep(K_MSEC(5000));
    if (!spi_is_ready_dt(&ads_spi)) {
        printk("SPI device not ready\n");
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

    ret = gpio_pin_configure_dt(&drdy_pin, GPIO_INPUT);
    if (ret) {
        printk("drdy configure failed: %d\n", ret);
        return 0;
    }

    ret = gpio_pin_configure_dt(&pwdwn_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        printk("FAIL: pwdwn configure failed: %d\n", ret);
        return 0;
    }
    if (!device_is_ready(pwdwn_pin.port)) { LOG_ERR("pwdwn port not ready"); return -1; }
    if (!device_is_ready(reset_pin.port))   { LOG_ERR("rst port not ready");   return -1; }
    if (!device_is_ready(start_pin.port)) { LOG_ERR("start port not ready"); return -1; }
    if (!device_is_ready(drdy_pin.port))  { LOG_ERR("drdy port not ready");  return -1; }
    gpio_pin_set_dt(&pwdwn_pin, 0);  // bring chip out of power down
    k_sleep(K_MSEC(100));
    printk("OK: PWDN high\n");

    /*
     * Keep START low; we will use SPI START command.
     */
    gpio_pin_set_dt(&start_pin, 1);

    /*
     * Hardware reset pulse.
     * If your DTS marks the line as active low, gpio_pin_set_dt()
     * handles that for you.
     */
    gpio_pin_set_dt(&reset_pin, 1);
    k_sleep(K_MSEC(100));
    gpio_pin_set_dt(&reset_pin, 0);
    k_sleep(K_MSEC(100));

    ret = ads_send_cmd(CMD_RESET);
    if (ret) {
        printk("CMD_RESET failed: %d\n", ret);
        return 0;
    }
    printk("OK: reset sequence done\n");
    k_sleep(K_MSEC(10));

    ret = ads_print_id();
    if (ret) {
        printk("ads_print_id failed: %d\n", ret);
        return 0;
    }

    ret = ads_configure_external_ch1_mode();
    if (ret) {
        printk("ads_configure_external_ch1_mode failed: %d\n", ret);
        return 0;
    }

    ret = ads_dump_key_regs();
    if (ret) {
        printk("ads_dump_key_regs failed: %d\n", ret);
        return 0;
    }

    ret = ads_send_cmd(CMD_START);
    if (ret) {
        printk("START cmd failed: %d\n", ret);
        return 0;
    }

    k_sleep(K_MSEC(4));

    ret = ble_init();
    if (ret) {
        return 0;
    }

    printk("Waiting for BLE RX command 'start'\n");
    printk("External test setup expected:\n");
    printk("  ESP32-S2 attenuated node -> CH1P\n");
    printk("  CH1N -> reference/return node\n");
    printk("  other channels off\n");

    while (1) {
        if (stream_requested && !stream_enabled && k_uptime_get() >= stream_start_at_ms) {
            stream_enabled = true;
            printk("Streaming enabled\n");
        }

        if (!stream_enabled) {
            k_sleep(K_MSEC(50));
            continue;
        }

        ret = ads_wait_drdy_low_timeout_ms(1000);
        if (ret) {
            printk("DRDY timeout/error: %d\n", ret);
            k_sleep(K_MSEC(10));
            continue;
        }

        ret = ads_read_frame_rdata(frame);
        if (ret) {
            printk("RDATA read error: %d\n", ret);
            k_sleep(K_MSEC(10));
            continue;
        }

        if ((sample_idx % STREAM_DECIMATION) == 0U) {
            build_binary_packet(pkt, sample_idx, frame);
            nus_send_binary(pkt, sizeof(pkt));
        }

        if ((sample_idx % 250U) == 0U) {
            int32_t ch1 = ads_decode24(&frame[3]);
            printk("sample=%lu status=%02X%02X%02X ch1=%ld\n",
                   (unsigned long)sample_idx,
                   frame[0], frame[1], frame[2],
                   (long)ch1);
        }

        sample_idx++;
    }
}
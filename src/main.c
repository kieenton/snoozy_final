/*
 * main.c — BLE NUS EEG window streamer for nRF54L15 (Zephyr RTOS)
 *
 * Commands (sent as plain ASCII over NUS):
 *   "start_meditation"     — stream meditation snippet
 *   "start_mind_wandering" — stream mind-wandering snippet
 *   "stop"                 — halt streaming
 *
 * Streaming structure (3 contexts, 10s apart):
 *
 *   t=0s  — context [0..2559]:
 *     Send 1: ch0[0..1279],    ch1[0..1279]
 *     Send 2: ch0[256..1535],  ch1[256..1535]
 *     Send 3: ch0[512..1791],  ch1[512..1791]
 *     Send 4: ch0[768..2047],  ch1[768..2047]
 *     Send 5: ch0[1024..2303], ch1[1024..2303]
 *     Send 6: ch0[1280..2559], ch1[1280..2559]
 *   wait 10s
 *   t=10s — context [2560..5119]: same window pattern offset by 2560
 *   wait 10s
 *   t=20s — context [5120..7679]: same window pattern offset by 5120
 *
 * Each send = 2560 floats as a flat CSV string:
 *   "ch0_0,ch0_1,...,ch0_1279,ch1_0,ch1_1,...,ch1_1279"
 * Split into 20-byte NUS notifications back-to-back.
 *
 * TypeScript reconstruction per send:
 *   const vals = data.split(',').map(Number);
 *   const ch0 = new Float32Array(vals.slice(0, 1280));
 *   const ch1 = new Float32Array(vals.slice(1280, 2560));
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <bluetooth/services/nus.h>

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "eeg_snippets.h"

LOG_MODULE_REGISTER(ble_streamer, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------
 * Device / advertising config
 * ---------------------------------------------------------------------- */
#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* -------------------------------------------------------------------------
 * Streaming config
 * ---------------------------------------------------------------------- */
#define BURST_INTERVAL_MS   10000   /* 10s between contexts                  */
#define NUS_MTU_FALLBACK    20      /* used until MTU exchange completes     */
#define TX_RETRY_DELAY_MS   5       /* back-off when TX queue full           */

#define N_CONTEXTS          3       /* 3 x 2560 samples = 7680 total         */
#define N_WINDOWS           6       /* windows per context                   */
#define WINDOW_SAMPLES      EEG_SNIPPET_WINDOW_SAMPLES   /* 1280             */
#define WINDOW_STEP         EEG_SNIPPET_WINDOW_STEP      /* 256              */
#define CONTEXT_SAMPLES     EEG_SNIPPET_CONTEXT_SAMPLES  /* 2560             */

/*
 * Each send = 2 channels x 1280 samples = 2560 floats.
 * Worst case per float: "-XXXX.XXX," = 10 chars.
 * Buffer: 2560 * 10 + 1 = 25601 bytes — kept as static global.
 */
#define FLOATS_PER_SEND   (EEG_SNIPPET_N_CHANNELS * WINDOW_SAMPLES)
#define CSV_BUF_SIZE      (FLOATS_PER_SEND * 10 + 3)
#define WINDOW_INTERVAL_MS  1666   /* ~10s / 6 windows */

/* -------------------------------------------------------------------------
 * State
 * ---------------------------------------------------------------------- */
static struct bt_conn *current_conn;
static bool            stream_enabled;

/* Pointer to active snippet's pre-windowed data [ch][win][sample] */
static const float (*active_windows)[EEG_SNIPPET_N_WINDOWS][EEG_SNIPPET_WINDOW_SAMPLES];

K_SEM_DEFINE(start_sem, 0, 1);

/* Static CSV buffer — too large for thread stack */
static char csv_buf[CSV_BUF_SIZE];

/* -------------------------------------------------------------------------
 * Streaming thread
 * ---------------------------------------------------------------------- */
#define STREAM_STACK_SIZE 2048
#define STREAM_PRIORITY   5
K_THREAD_STACK_DEFINE(stream_stack, STREAM_STACK_SIZE);
static struct k_thread stream_thread_data;

/*
 * nus_send_all() — splits str into ATT-MTU-sized chunks, sends back-to-back.
 * Uses the negotiated MTU (minus 3-byte ATT header); falls back to 20 bytes.
 */
static int nus_send_all(struct bt_conn *conn, const char *str, size_t len)
{
    uint16_t mtu = bt_gatt_get_mtu(conn);
    size_t max_chunk = (mtu > 3) ? (mtu - 3) : NUS_MTU_FALLBACK;
    size_t offset = 0;

    while (offset < len) {
        size_t chunk = MIN(len - offset, max_chunk);
        int err;

        do {
            err = bt_nus_send(conn, (const uint8_t *)(str + offset), chunk);
            if (err == -EAGAIN || err == -ENOMEM) {
                k_msleep(TX_RETRY_DELAY_MS);
            } else if (err) {
                LOG_ERR("bt_nus_send error %d", err);
                return err;
            }
        } while (err != 0);

        offset += chunk;
    }
    return 0;
}

/*
 * build_window_csv() — formats one pre-windowed window for both channels into csv_buf.
 *
 * win: window index (0–5)
 *
 * Layout: ch0[0..1279], ch1[0..1279]
 */
static size_t build_window_csv(int win)
{
    size_t written = 0;

    csv_buf[written++] = '[';

    for (int ch = 0; ch < EEG_SNIPPET_N_CHANNELS; ch++) {
        for (int s = 0; s < WINDOW_SAMPLES; s++) {
            int n = snprintf(csv_buf + written, CSV_BUF_SIZE - written,
                             "%s%.3f",
                             (written > 1) ? "," : "",
                             (double)active_windows[ch][win][s]);
            if (n < 0 || (size_t)n >= CSV_BUF_SIZE - written) {
                break;
            }
            written += n;
        }
    }

    csv_buf[written++] = ']';
    csv_buf[written] = '\0';
    return written;
}

static void stream_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sem_take(&start_sem, K_FOREVER);
        LOG_INF("Streaming started");

        for (int win = 0; win < EEG_SNIPPET_N_WINDOWS && stream_enabled && current_conn; win++) {

            LOG_INF("Window %d/%d", win, EEG_SNIPPET_N_WINDOWS - 1);

            size_t len = build_window_csv(win);

            if (nus_send_all(current_conn, csv_buf, len) != 0) {
                goto stream_stopped;
            }

            if (win < EEG_SNIPPET_N_WINDOWS - 1 && stream_enabled && current_conn) {
                k_msleep(WINDOW_INTERVAL_MS);
            }
        }

        if (stream_enabled) {
            LOG_INF("All contexts sent -- streaming complete");
            stream_enabled = false;
        }

stream_stopped:
        LOG_INF("Streaming stopped");
    }
}

/* -------------------------------------------------------------------------
 * NUS callbacks
 * ---------------------------------------------------------------------- */
static void nus_rx_cb(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    if (len >= 16 && memcmp(data, "start_meditation", 16) == 0) {
        if (!stream_enabled) {
            active_windows = eeg_meditation_windows;
            stream_enabled = true;
            LOG_INF("Snippet: meditation");
            k_sem_give(&start_sem);
        }
    } else if (len >= 20 && memcmp(data, "start_mind_wandering", 20) == 0) {
        if (!stream_enabled) {
            active_windows = eeg_mind_wandering_windows;
            stream_enabled = true;
            LOG_INF("Snippet: mind_wandering");
            k_sem_give(&start_sem);
        }
    } else if (len >= 4 && memcmp(data, "stop", 4) == 0) {
        LOG_INF("Stop command received");
        stream_enabled = false;
    } else {
        LOG_WRN("Unknown command (len=%u)", len);
    }
}

static struct bt_nus_cb nus_cb = {
    .received = nus_rx_cb,
};

/* -------------------------------------------------------------------------
 * Advertising data
 * ---------------------------------------------------------------------- */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* -------------------------------------------------------------------------
 * BT connection callbacks
 * ---------------------------------------------------------------------- */
static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                             struct bt_gatt_exchange_params *params)
{
    if (!err) {
        LOG_INF("MTU exchanged: %u bytes/notification", bt_gatt_get_mtu(conn) - 3);
    } else {
        LOG_WRN("MTU exchange failed (err %u), using %u bytes", err, NUS_MTU_FALLBACK);
    }
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
    .func = mtu_exchange_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    LOG_INF("Connected");
    bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    stream_enabled = false;

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    int err = bt_le_adv_start(
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
                        BT_GAP_ADV_FAST_INT_MIN_2,
                        BT_GAP_ADV_FAST_INT_MAX_2,
                        NULL),
        ad, ARRAY_SIZE(ad),
        sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising restart failed (err %d)", err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* -------------------------------------------------------------------------
 * Entry point
 * ---------------------------------------------------------------------- */
int main(void)
{
    int err;

    LOG_INF("BLE EEG streamer starting");

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return err;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("bt_nus_init failed (err %d)", err);
        return err;
    }

    err = bt_le_adv_start(
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN,
                        BT_GAP_ADV_FAST_INT_MIN_2,
                        BT_GAP_ADV_FAST_INT_MAX_2,
                        NULL),
        ad, ARRAY_SIZE(ad),
        sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("bt_le_adv_start failed (err %d)", err);
        return err;
    }
    LOG_INF("Advertising as \"%s\"", DEVICE_NAME);

    k_thread_create(&stream_thread_data, stream_stack,
                     K_THREAD_STACK_SIZEOF(stream_stack),
                     stream_thread_fn,
                     NULL, NULL, NULL,
                     STREAM_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&stream_thread_data, "ble_stream");

    return 0;
}
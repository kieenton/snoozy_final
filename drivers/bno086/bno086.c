/*
 * bno086.c
 *
 * Initialises the BNO086 via the Ceva sh2 library and enables three reports:
 *   - Rotation Vector          (quaternion, 100 Hz)
 *   - Accelerometer            (100 Hz)
 *   - Gyroscope Calibrated     (100 Hz)
 *
 * Call bno086_init() once at startup, then call bno086_service() from your
 * main loop (or a dedicated thread) as often as possible.
 * Parsed data is delivered via the bno086_data_cb callback.
 */

#include "bno086.h"
#include "bno086_hal.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/* ------------------------------------------------------------------ */
/* Configuration                                                        */
/* ------------------------------------------------------------------ */

/* Report intervals in microseconds */
#define ROTATION_VECTOR_INTERVAL_US   10000U   /* 100 Hz */
#define ACCEL_INTERVAL_US             10000U   /* 100 Hz */
#define GYRO_INTERVAL_US              10000U   /* 100 Hz */

/* ------------------------------------------------------------------ */
/* Internal state                                                       */
/* ------------------------------------------------------------------ */

static bno086_data_cb_t user_cb;
static bool             sh2_ready;

/* ------------------------------------------------------------------ */
/* sh2 event callback (reset detection)                                 */
/* ------------------------------------------------------------------ */

static void on_sh2_event(void *cookie, sh2_AsyncEvent_t *event)
{
    ARG_UNUSED(cookie);

    if (event->eventId == SH2_RESET) {
        printk("BNO086: device reset detected, re-enabling reports\n");
        sh2_ready = false;   /* triggers re-init in bno086_service() */
    }
}

/* ------------------------------------------------------------------ */
/* sh2 sensor event callback                                            */
/* ------------------------------------------------------------------ */

static void on_sensor_event(void *cookie, sh2_SensorEvent_t *event)
{
    ARG_UNUSED(cookie);

    sh2_SensorValue_t val;

    if (sh2_decodeSensorEvent(&val, event) != SH2_OK) {
        return;
    }

    if (user_cb == NULL) {
        return;
    }

    bno086_data_t out;
    memset(&out, 0, sizeof(out));
    out.timestamp_us = val.timestamp;

    switch (val.sensorId) {

    case SH2_ROTATION_VECTOR:
        out.type     = BNO086_REPORT_ROTATION;
        out.quat.i   = val.un.rotationVector.i;
        out.quat.j   = val.un.rotationVector.j;
        out.quat.k   = val.un.rotationVector.k;
        out.quat.real = val.un.rotationVector.real;
        out.quat.accuracy = val.un.rotationVector.accuracy;
        user_cb(&out);
        break;

    case SH2_ACCELEROMETER:
        out.type    = BNO086_REPORT_ACCEL;
        out.accel.x = val.un.accelerometer.x;
        out.accel.y = val.un.accelerometer.y;
        out.accel.z = val.un.accelerometer.z;
        user_cb(&out);
        break;

    case SH2_GYROSCOPE_CALIBRATED:
        out.type   = BNO086_REPORT_GYRO;
        out.gyro.x = val.un.gyroscope.x;
        out.gyro.y = val.un.gyroscope.y;
        out.gyro.z = val.un.gyroscope.z;
        user_cb(&out);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Internal: enable the three sensor reports                            */
/* ------------------------------------------------------------------ */

static int enable_reports(void)
{
    int ret;

    sh2_SensorConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    /* Rotation Vector */
    cfg.reportInterval_us = ROTATION_VECTOR_INTERVAL_US;
    ret = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &cfg);
    if (ret != SH2_OK) {
        printk("BNO086: enable rotation vector failed: %d\n", ret);
        return ret;
    }

    /* Accelerometer */
    cfg.reportInterval_us = ACCEL_INTERVAL_US;
    ret = sh2_setSensorConfig(SH2_ACCELEROMETER, &cfg);
    if (ret != SH2_OK) {
        printk("BNO086: enable accel failed: %d\n", ret);
        return ret;
    }

    /* Gyroscope calibrated */
    cfg.reportInterval_us = GYRO_INTERVAL_US;
    ret = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
    if (ret != SH2_OK) {
        printk("BNO086: enable gyro failed: %d\n", ret);
        return ret;
    }

    printk("BNO086: reports enabled\n");
    return 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

int bno086_init(bno086_data_cb_t cb)
{
    int ret;

    user_cb   = cb;
    sh2_ready = false;

    ret = sh2_open(bno086_hal_get(), on_sh2_event, NULL);
    if (ret != SH2_OK) {
        printk("BNO086: sh2_open failed: %d\n", ret);
        return ret;
    }

    /* Print product IDs for verification */
    sh2_ProductIds_t ids;
    ret = sh2_getProdIds(&ids);
    if (ret == SH2_OK) {
        for (int i = 0; i < ids.numEntries; i++) {
            printk("BNO086 part %lu ver %lu.%lu.%lu build %lu\n",
                   (unsigned long)ids.entry[i].swPartNumber,
                   (unsigned long)ids.entry[i].swVersionMajor,
                   (unsigned long)ids.entry[i].swVersionMinor,
                   (unsigned long)ids.entry[i].swVersionPatch,
                   (unsigned long)ids.entry[i].swBuildNumber);
        }
    }

    sh2_setSensorCallback(on_sensor_event, NULL);

    ret = enable_reports();
    if (ret) {
        return ret;
    }

    sh2_ready = true;
    printk("BNO086: init complete\n");
    return 0;
}

void bno086_service(void)
{
    if (!sh2_ready) {
        /* Device reset — re-open and re-enable reports */
        printk("BNO086: re-initialising after reset\n");
        sh2_close();
        k_sleep(K_MSEC(50));

        if (sh2_open(bno086_hal_get(), on_sh2_event, NULL) == SH2_OK) {
            sh2_setSensorCallback(on_sensor_event, NULL);
            if (enable_reports() == 0) {
                sh2_ready = true;
            }
        }
        return;
    }

    sh2_service();
}
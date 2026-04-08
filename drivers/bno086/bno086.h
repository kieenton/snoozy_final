#ifndef BNO086_H
#define BNO086_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Data types delivered to the application callback                     */
/* ------------------------------------------------------------------ */

typedef enum {
    BNO086_REPORT_ROTATION,
    BNO086_REPORT_ACCEL,
    BNO086_REPORT_GYRO,
} bno086_report_type_t;

typedef struct {
    float i, j, k, real;
    float accuracy;   /* estimated heading accuracy in radians */
} bno086_quat_t;

typedef struct {
    float x, y, z;   /* m/s^2 */
} bno086_accel_t;

typedef struct {
    float x, y, z;   /* rad/s */
} bno086_gyro_t;

typedef struct {
    bno086_report_type_t type;
    uint64_t             timestamp_us;
    union {
        bno086_quat_t  quat;
        bno086_accel_t accel;
        bno086_gyro_t  gyro;
    };
} bno086_data_t;

/* Callback type — called from bno086_service() context */
typedef void (*bno086_data_cb_t)(const bno086_data_t *data);

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/**
 * Initialise the BNO086.
 * Call once after SPI is ready.
 *
 * @param cb  Callback invoked for each received sensor report.
 *            May be NULL if you don't need data yet.
 * @return 0 on success, negative on error.
 */
int bno086_init(bno086_data_cb_t cb);

/**
 * Service the BNO086 — call as frequently as possible from your main
 * loop or a dedicated thread.  Internally calls sh2_service() which
 * drains the SPI receive buffer and fires the data callback.
 */
void bno086_service(void);

#endif /* BNO086_H */
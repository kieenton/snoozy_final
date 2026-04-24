#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include "../drivers/bno086/bno086.h"

LOG_MODULE_REGISTER(bno_test, LOG_LEVEL_DBG);

static void on_bno_data(const bno086_data_t *data)
{
    switch (data->type) {

    case BNO086_REPORT_ROTATION:
        printk("QUAT i=%.4f j=%.4f k=%.4f r=%.4f acc=%.4f\n",
               (double)data->quat.i,
               (double)data->quat.j,
               (double)data->quat.k,
               (double)data->quat.real,
               (double)data->quat.accuracy);
        break;

    case BNO086_REPORT_ACCEL:
        printk("ACCEL x=%.3f y=%.3f z=%.3f m/s^2\n",
               (double)data->accel.x,
               (double)data->accel.y,
               (double)data->accel.z);
        break;

    case BNO086_REPORT_GYRO:
        printk("GYRO  x=%.3f y=%.3f z=%.3f rad/s\n",
               (double)data->gyro.x,
               (double)data->gyro.y,
               (double)data->gyro.z);
        break;
    }
}

int main(void)
{
    int ret;

    k_sleep(K_MSEC(2000));
    printk("BNO086 SPI test starting\n");

    ret = bno086_init(on_bno_data);
    if (ret) {
        printk("FAIL: bno086_init returned %d\n", ret);
        printk("Check: SPI wiring, RST pin, BOOT pin tied high\n");
        return 0;
    }

    printk("OK: bno086_init complete — servicing\n");

    while (1) {
        bno086_service();
        k_sleep(K_MSEC(10));
    }
}

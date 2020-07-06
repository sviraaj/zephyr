#ifndef __APP_CFG_H__
#define __APP_CFG_H__

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <sys/printk.h>
#include "app_modem.h"
#include "config_params.h"
#include "app_log.h"

struct app_config
{
    struct device *modem_dev;
    struct device *imu_dev;
    struct device *env1_dev;
    struct device *env2_dev;
    struct device *flash_dev;
    struct device *pwm_dev;
    struct device *uart1_dev;
    struct device *gpio0_dev;
    struct device *gpio1_dev;
    struct device *gpio_p0_dev;
    struct device *display_dev;

    struct global_params __aligned(4) g_params;
    struct device_params __aligned(4) dev_params;

    struct modem_cfg mdm_cfg;
    //struct display_cfg disp_cfg;
    //struct compressor_cfg comp_cfg;
};

int app_init(struct app_config *app_cfg);
void get_global_params(struct global_params** g_params);
void get_device_params(struct device_params** dev_params);
void led_toggle();

#endif

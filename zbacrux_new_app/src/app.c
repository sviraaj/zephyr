#include <zephyr.h>
#include <kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_cfg.h"
#include "dev_data.h"
#include <drivers/gpio.h>

#define APP_SYS_VERSION      "v1.0"
//#define MDM_THREAD_STACK_SIZE 8192
//#define MDM_THREAD_PRIORITY 7

#define MAX_RETRY_COUNT        5U

//K_THREAD_STACK_DEFINE(mdm_thread_stack, MDM_THREAD_STACK_SIZE);

static void data_upload_cb(struct k_timer *dummy);

K_TIMER_DEFINE(data_upload_timer, data_upload_cb, NULL);

//struct k_thread mdm_thread_data;
//k_tid_t mdm_tid;

//static struct k_timer mdm_timer;

static struct app_config app_cfg;

static struct k_sem sem_mdm_trigger;

void get_global_params(struct global_params** g_params)
{
    *g_params = &(app_cfg.g_params);
}

void get_device_params(struct device_params** dev_params)
{
    *dev_params = &(app_cfg.dev_params);
}

static void data_upload_cb(struct k_timer *dummy)
{
    k_sem_give(&sem_mdm_trigger);
}

void led_toggle()
{
    gpio_pin_write(app_cfg.gpio_p0_dev, 0, 0);
    k_sleep(50);
    gpio_pin_write(app_cfg.gpio_p0_dev, 0, 1);
}

#if 0
void mdm_thread(void *param, void *arg1, void *arg2)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(param);
    int ret = 0;
    int retry_cnt = MAX_RETRY_COUNT;

    while (retry_cnt--)
    {
        ret = login_device(app_cfg.modem_dev, &app_cfg.mdm_cfg);
        if (ret != 0)
        {
            error_log("Cloud login failed");
        }
    }

    /* one shot timer */
    k_timer_start(&data_upload_timer, app_cfg.dev_params.upload_interval, 0);

    while(1)
    {
        k_sem_take(&sem_mdm_trigger, K_FOREVER);

        ret = send_device_data(app_cfg.modem_dev, app_cfg.mdm_cfg);
        if (ret != 0)
        {
            error_log("cloud upload failed");
        }

        /* one shot timer */
        k_timer_start(&data_upload_timer, app_cfg.dev_params.upload_interval, 0);
    }
}
#endif

int main(void)
{
    int ret = 0;
    int retry_cnt = 1;//MAX_RETRY_COUNT;

    app_cfg.g_params.sys_version = APP_SYS_VERSION;

    ret = app_init(&app_cfg);
    if (ret != 0)
    {
        error_log("app init failed");
    }

    gpio_pin_configure(app_cfg.gpio_p0_dev, 0, GPIO_DIR_OUT);

    k_sem_init(&sem_mdm_trigger, 1, 1);

#if 0
	mdm_tid = k_thread_create(&mdm_thread_data, mdm_thread_stack,
				 K_THREAD_STACK_SIZEOF(mdm_thread_stack),
				 mdm_thread, NULL, NULL, NULL,
				 MDM_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!mdm_tid)
    {
        error_log("mdm thread create failed!");
    }
#endif

    while (retry_cnt--)
    {
        info_log("login device");

        ret = login_device(app_cfg.modem_dev, &app_cfg.mdm_cfg);
        if (ret != 0)
        {
            error_log("Cloud login failed");
        }

        k_sleep(5000);
    }

    /* one shot timer */
    //k_timer_start(&data_upload_timer, K_MSEC(app_cfg.dev_params.upload_interval), 0);

    while(1)
    {
        k_sem_take(&sem_mdm_trigger, K_FOREVER);

        info_log("send device data");

        ret = send_device_data(app_cfg.modem_dev, &app_cfg.mdm_cfg);
        if (ret != 0)
        {
            error_log("cloud upload failed");
            fetch_unit_rollback();
        }

        /* one shot timer */
        k_timer_start(&data_upload_timer, K_MSEC(app_cfg.dev_params.upload_interval), 0);
    }

    return 0;
}

#ifndef __APP_MODEM_H__
#define __APP_MODEM_H__

#include "config_params.h"

/* TODO Move important defines to prj.cfg */
#define MAX_URL_SIZE 512U
#define MAX_BODY_SIZE 4000U
#define MAX_MISC_SIZE 255U
#define MAX_RECV_RESP_BUF 2048U

struct modem_cfg
{
    struct device_params *dev_params;
    struct global_params *g_params;

    struct k_sem serv_resp_sem;
};

int login_device(struct device* modem_dev, struct modem_cfg *mdm_cfg);
int send_device_data(struct device* modem_dev, struct modem_cfg *mdm_cfg);

#endif

#ifndef __APP_COMPRESSOR_H__
#define __APP_COMPRESSOR_H__

#include "app_cfg.h"

#define UART_5V_ENABLE_PIN         12
#define COMPRESSOR_UART_INT_PRIO   6
#define MIN_RPM                    1800
#define MAX_RPM                    3000

struct compressor_rx_data
{
    u16_t comp_cur_speed;
    u16_t comp_cur_current;
    u16_t bus_volt;
    u8_t drv_temp;
    u8_t drv_err;
    u8_t drv_cnd;
};

void compressor_set_target_temp(int32_t temp);
int compressor_init(struct device* compressor_dev, struct device* gpio);
void get_compressor_data(struct compressor_rx_data** comp);

#endif

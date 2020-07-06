#ifndef __APP_IO_H__
#define __APP_IO_H__

#include <zephyr.h>
#include <drivers/gpio.h>
#include <device.h>
#include "app_log.h"

int app_io_init(struct device* gpio, struct device* pwm);

#endif

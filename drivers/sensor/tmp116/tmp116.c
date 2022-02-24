/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tmp116

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <kernel.h>

#include "tmp116.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(TMP116);

static int tmp116_reg_read(struct device *dev, u8_t reg, u16_t *val)
{
	struct tmp116_data *drv_data = dev->driver_data;

	if (i2c_burst_read(drv_data->i2c, drv_data->i2c_slave_addr, reg, (u8_t *)val, 2)
	    < 0) {
		return -EIO;
	}

	*val = sys_be16_to_cpu(*val);

	return 0;
}

/**
 * @brief Check the Device ID
 *
 * @param[in]   dev     Pointer to the device structure
 *
 * @retval 0 On success
 * @retval -EIO Otherwise
 */
static inline int tmp116_device_id_check(struct device *dev)
{
	u16_t value;

	if (tmp116_reg_read(dev, TMP116_REG_DEVICE_ID, &value) != 0) {
		LOG_ERR("%s: Failed to get Device ID register!",
			dev->name);
		return -EIO;
	}

	if ((value != TMP116_DEVICE_ID) && (value != TMP117_DEVICE_ID)) {
		LOG_ERR("%s: Failed to match the device IDs!",
			dev->name);
		return -EINVAL;
	}

	return 0;
}

static int tmp116_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct tmp116_data *drv_data = dev->driver_data;
	u16_t value;
	int rc;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_AMBIENT_TEMP);

	/* clear sensor values */
	drv_data->sample = 0U;

	/* Get the most recent temperature measurement */
	rc = tmp116_reg_read(dev, TMP116_REG_TEMP, &value);
	if (rc < 0) {
		LOG_ERR("%s: Failed to read from TEMP register!",
            dev->name);
		return rc;
	}

	/* store measurements to the driver */
	drv_data->sample = (s16_t)value;

	return 0;
}

static int tmp116_channel_get(struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct tmp116_data *drv_data = dev->driver_data;
	s32_t tmp;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	/*
	 * See datasheet "Temperature Results and Limits" section for more
	 * details on processing sample data.
	 */
	tmp = ((s16_t)drv_data->sample * (s32_t)TMP116_RESOLUTION) / 10;
	val->val1 = tmp / 1000000; /* uCelsius */
	val->val2 = tmp % 1000000;

	return 0;
}

static const struct sensor_driver_api tmp116_driver_api = {
	.sample_fetch = tmp116_sample_fetch,
	.channel_get = tmp116_channel_get
};

#define TMP116_DEVICE(inst)				                                    \
static int tmp116_##inst##_init(struct device *dev)			                \
{                                                                           \
	struct tmp116_data *data = dev->driver_data;                            \
	data->i2c = device_get_binding(DT_INST_BUS_LABEL(inst));                \
	if (!data->i2c) {                                                       \
		LOG_INF("i2c master not found: %s",                                 \
			    DT_INST_BUS_LABEL(inst));                                   \
		return -EINVAL;                                                     \
	}                                                                       \
	data->i2c_slave_addr = DT_INST_REG_ADDR(inst);                          \
	if (tmp116_device_id_check(dev) < 0) {                                  \
		return -EINVAL;                                                     \
	}                                                                       \
    return 0;                                                               \
}                                                                           \
                                                                            \
static struct tmp116_data tmp116_##inst##_data;                             \
                                                                            \
DEVICE_AND_API_INIT(tmp116_##inst,				                            \
          DT_INST_LABEL(inst),		                                        \
          tmp116_##inst##_init,					                            \
          &tmp116_##inst##_data,				                            \
          NULL,              				                                \
          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,		                    \
          &tmp116_driver_api);


DT_INST_FOREACH_STATUS_OKAY(TMP116_DEVICE)

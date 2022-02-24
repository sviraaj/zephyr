/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adxl357

#include <kernel.h>
#include <string.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <stdlib.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "adxl357.h"

LOG_MODULE_REGISTER(ADXL357, CONFIG_SENSOR_LOG_LEVEL);

static int adxl357_bus_access(struct device *dev, u8_t reg,
			      void *data, size_t length)
{
	struct adxl357_data *adxl357_data = dev->driver_data;

#ifdef CONFIG_ADXL357_SPI
	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1
		}, {
			.buf = data,
			.len = length
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
	};

	if (reg & ADXL357_READ) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		tx.count = 1;

		return spi_transceive(adxl357_data->bus,
				      &adxl357_data->spi_cfg, &tx, &rx);
	}

	tx.count = 2;

	return spi_write(adxl357_data->bus, &adxl357_data->spi_cfg, &tx);
#elif CONFIG_ADXL357_I2C
	const struct adxl357_dev_config *cfg = dev->config_info;

	if (reg & ADXL357_READ) {
		return i2c_burst_read(adxl357_data->bus, cfg->i2c_addr,
				      ADXL357_TO_I2C_REG(reg),
				      (u8_t *) data, length);
	} else {
		if (length != 1) {
			return -EINVAL;
		}

		return i2c_reg_write_byte(adxl357_data->bus, cfg->i2c_addr,
					  ADXL357_TO_I2C_REG(reg),
					  *(u8_t *)data);
	}

#endif
}

/**
 * Read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_reg_read(struct device *dev,
			     u8_t reg_addr,
			     u8_t *reg_data)
{
	return adxl357_bus_access(dev, ADXL357_REG_READ(reg_addr), reg_data, 1);
}

/**
 * Multibyte read from device. A register read begins with the address
 * and autoincrements for each additional byte in the transfer.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @param count - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_reg_read_multiple(struct device *dev,
				      u8_t reg_addr,
				      u8_t *reg_data,
				      u16_t count)
{
	return adxl357_bus_access(dev, ADXL357_REG_READ(reg_addr),
				  reg_data, count);
}

/**
 * Write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_reg_write(struct device *dev,
			      u8_t reg_addr,
			      u8_t reg_data)
{
	LOG_DBG("[0x%X] = 0x%X", reg_addr, reg_data);

	return adxl357_bus_access(dev, ADXL357_REG_WRITE(reg_addr),
				  &reg_data, 1);
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int adxl357_reg_write_mask(struct device *dev,
			       u8_t reg_addr,
			       u32_t mask,
			       u8_t data)
{
	int ret;
	u8_t tmp;

	ret = adxl357_reg_read(dev, reg_addr, &tmp);
	if (ret) {
		return ret;
	}

	tmp &= ~mask;
	tmp |= data;

	return adxl357_reg_write(dev, reg_addr, tmp);
}

/**
 * Set the threshold for activity detection for a single axis
 * @param dev - The device structure.
 * @param axis_reg_h - The high part of the activity register.
 * @param act - The activity config structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_activity_threshold(struct device *dev, u8_t axis_reg_h,
				   const struct adxl357_activity_threshold *act)
{
	int ret;
	u8_t val;

	ret = adxl357_reg_write(dev, axis_reg_h++, act->thresh >> 3);
	if (ret) {
		return ret;
	}

	switch (axis_reg_h) {
	case ADXL357_X_THRESH_ACT_L:
	case ADXL357_X_THRESH_INACT_L:
	case ADXL357_X_THRESH_ACT2_L:
		val = (act->thresh << 5) | (act->referenced << 1) | act->enable;
		break;
	default:
		val = (act->thresh << 5) | act->enable;
	}

	return adxl357_reg_write(dev, axis_reg_h, val);
}

/**
 * Set the threshold for activity detection for all 3-axis
 * @param dev - The device structure.
 * @param axis_reg_h - The high part of the activity register.
 * @param act - The activity config structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_activity_threshold_xyz(struct device *dev,
				u8_t axis_reg_h,
				const struct adxl357_activity_threshold *act)
{
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = adxl357_set_activity_threshold(dev, axis_reg_h, act);
		if (ret) {
			return ret;
		}
		axis_reg_h += 2U;
	}

	return 0;
}

/**
 * Set the mode of operation.
 * @param dev - The device structure.
 * @param op_mode - Mode of operation.
 *		Accepted values: ADXL357_STANDBY
 *				 ADXL357_WAKE_UP
 *				 ADXL357_INSTANT_ON
 *				 ADXL357_FULL_BW_MEASUREMENT
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_op_mode(struct device *dev, enum adxl357_op_mode op_mode)
{
	return adxl357_reg_write_mask(dev, ADXL357_POWER_CTL,
				      ADXL357_POWER_CTL_MODE_MSK,
				      ADXL357_POWER_CTL_MODE(op_mode));
}

/**
 * Autosleep. When set to 1, autosleep is enabled, and the device enters
 * wake-up mode automatically upon detection of inactivity.
 * @param dev - The device structure.
 * @param enable - Accepted values: true
 *				    false
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_autosleep(struct device *dev, bool enable)
{
	return adxl357_reg_write_mask(dev, ADXL357_MEASURE,
				      ADXL357_MEASURE_AUTOSLEEP_MSK,
				      ADXL357_MEASURE_AUTOSLEEP_MODE(enable));
}

/**
 * Select the desired output signal bandwidth.
 * @param dev - The device structure.
 * @param bw - bandwidth.
 *		Accepted values: ADXL357_BW_200HZ
 *				 ADXL357_BW_400HZ
 *				 ADXL357_BW_800HZ
 *				 ADXL357_BW_1600HZ
 *				 ADXL357_BW_3200HZ
 *				 ADXL357_BW_LPF_DISABLED
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_bandwidth(struct device *dev, enum adxl357_bandwidth bw)
{
	int ret;
	u8_t mask;

	if (bw == ADXL357_BW_LPF_DISABLED) {
		mask = ADXL357_POWER_CTL_LPF_DIS_MSK;
	} else {
		mask = 0U;
	}

	ret = adxl357_reg_write_mask(dev, ADXL357_POWER_CTL,
				     ADXL357_POWER_CTL_LPF_DIS_MSK, mask);
	if (ret) {
		return ret;
	}

	return adxl357_reg_write_mask(dev, ADXL357_MEASURE,
				      ADXL357_MEASURE_BANDWIDTH_MSK,
				      ADXL357_MEASURE_BANDWIDTH_MODE(bw));
}

/**
 * Select the desired high-pass filter coner.
 * @param dev - The device structure.
 * @param bw - bandwidth.
 *		Accepted values: ADXL357_HPF_CORNER_0
 *				 ADXL357_HPF_CORNER_1
 *				 ADXL357_HPF_CORNER_2
 *				 ADXL357_HPF_CORNER_3
 *				 ADXL357_HPF_DISABLED
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_hpf_corner(struct device *dev, enum adxl357_hpf_corner c)
{

	int ret;
	u8_t mask;

	if (c == ADXL357_HPF_DISABLED) {
		mask = ADXL357_POWER_CTL_HPF_DIS_MSK;
	} else {
		mask = 0U;
	}

	ret = adxl357_reg_write_mask(dev, ADXL357_POWER_CTL,
				     ADXL357_POWER_CTL_HPF_DIS_MSK, mask);
	if (ret) {
		return ret;
	}

	return adxl357_reg_write(dev, ADXL357_HPF, ADXL357_HPF_CORNER(c));
}


/**
 * Link/Loop Activity Processing.
 * @param dev - The device structure.
 * @param mode - Mode of operation.
 *		Accepted values: ADXL357_DEFAULT
 *				 ADXL357_LINKED
 *				 ADXL357_LOOPED
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_act_proc_mode(struct device *dev,
				     enum adxl357_act_proc_mode mode)
{
	return adxl357_reg_write_mask(dev, ADXL357_MEASURE,
				      ADXL357_MEASURE_LINKLOOP_MSK,
				      ADXL357_MEASURE_LINKLOOP_MODE(mode));
}

/**
 * Set Output data rate.
 * @param dev - The device structure.
 * @param odr - Output data rate.
 *		Accepted values: ADXL357_ODR_400HZ
 *				 ADXL357_ODR_800HZ
 *				 ADXL357_ODR_1600HZ
 *				 ADXL357_ODR_3200HZ
 *				 ADXL357_ODR_6400HZ
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_odr_lpf(struct device *dev, enum adxl357_odr_lpf odr_lpf)
{
	return adxl357_reg_write_mask(dev, ADXL357_TIMING,
				      ADXL357_TIMING_ODR_MSK,
				      ADXL357_TIMING_ODR_MODE(odr));
}

/**
 * Select instant on threshold
 * @param dev - The device structure.
 * @param mode - 0 = low threshold, 1 = high threshold.
 *		Accepted values: ADXL357_INSTANT_ON_LOW_TH
 *				 ADXL357_INSTANT_ON_HIGH_TH
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_instant_on_th(struct device *dev,
				  enum adxl357_instant_on_th_mode mode)
{
	return adxl357_reg_write_mask(dev, ADXL357_POWER_CTL,
				ADXL357_POWER_CTL_INSTANT_ON_TH_MSK,
				ADXL357_POWER_CTL_INSTANT_ON_TH_MODE(mode));
}

/**
 * Set the Timer Rate for Wake-Up Mode.
 * @param dev - The device structure.
 * @param wur - wake up mode rate
 *		Accepted values: ADXL357_WUR_52ms
 *				 ADXL357_WUR_104ms
 *				 ADXL357_WUR_208ms
 *				 ADXL357_WUR_512ms
 *				 ADXL357_WUR_2048ms
 *				 ADXL357_WUR_4096ms
 *				 ADXL357_WUR_8192ms
 *				 ADXL357_WUR_24576ms
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_wakeup_rate(struct device *dev,
				enum adxl357_wakeup_rate wur)
{
	return adxl357_reg_write_mask(dev, ADXL357_TIMING,
				      ADXL357_TIMING_WAKE_UP_RATE_MSK,
				      ADXL357_TIMING_WAKE_UP_RATE_MODE(wur));
}

/**
 * Set the activity timer
 * @param dev - The device structure.
 * @param time - The value set in this register.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_activity_time(struct device *dev, u8_t time)
{
	return adxl357_reg_write(dev, ADXL357_TIME_ACT, time);
}

/**
 * Set the inactivity timer
 * @param dev - The device structure.
 * @param time - is the 16-bit value set by the TIME_INACT_L register
 *		 (eight LSBs) and the TIME_INACT_H register (eight MSBs).
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_inactivity_time(struct device *dev, u16_t time)
{
	int ret;

	ret = adxl357_reg_write(dev, ADXL357_TIME_INACT_H, time >> 8);
	if (ret) {
		return ret;
	}

	return adxl357_reg_write(dev, ADXL357_TIME_INACT_L, time & 0xFF);
}

/**
 * Set the filter settling period.
 * @param dev - The device structure.
 * @param mode - settle period
 *		Accepted values: ADXL357_FILTER_SETTLE_370
 *				 ADXL357_FILTER_SETTLE_16
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_set_filter_settle(struct device *dev,
				  enum adxl357_filter_settle mode)
{
	return adxl357_reg_write_mask(dev, ADXL357_POWER_CTL,
				      ADXL357_POWER_CTL_FIL_SETTLE_MSK,
				      ADXL357_POWER_CTL_FIL_SETTLE_MODE(mode));
}

/**
 * Configure the INT1 and INT2 interrupt pins.
 * @param dev - The device structure.
 * @param int1 -  INT1 interrupt pins.
 * @param int2 -  INT2 interrupt pins.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_interrupt_config(struct device *dev,
				 u8_t int1,
				 u8_t int2)
{
	int ret;

	ret = adxl357_reg_write(dev, ADXL357_INT1_MAP, int1);
	if (ret) {
		return ret;
	}

	return  adxl357_reg_write(dev, ADXL357_INT2_MAP, int2);

}

/**
 * Get the STATUS, STATUS2, FIFO_ENTRIES and FIFO_ENTRIES2 registers data
 * @param dev - The device structure.
 * @param status1 - Data stored in the STATUS1 register
 * @param status2 - Data stored in the STATUS2 register
 * @param fifo_entries - Number of valid data samples present in the
 *			 FIFO buffer (0 to 512)
 * @return 0 in case of success, negative error code otherwise.
 */
int adxl357_get_status(struct device *dev,
			   u8_t *status1,
			   u8_t *status2,
			   u16_t *fifo_entries)
{
	u8_t buf[4], length = 1U;
	int ret;

	if (status2) {
		length++;
	}

	if (fifo_entries) {
		length += 2U;
	}

	ret = adxl357_reg_read_multiple(dev, ADXL357_STATUS_1, buf, length);

	*status1 = buf[0];

	if (status2) {
		*status2 = buf[1];
	}

	if (fifo_entries) {
		*fifo_entries = ((buf[2] & 0x3) << 8) | buf[3];
	}

	return ret;
}

/**
 * Software reset.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_reset(struct device *dev)
{
	int ret;

	ret = adxl357_set_op_mode(dev, ADXL357_STANDBY);
	if (ret) {
		return ret;
	}
	/* Writing code 0x52 resets the device */
	ret = adxl357_reg_write(dev, ADXL357_RESET, ADXL357_RESET_CODE);
	k_sleep(K_MSEC(1000));

	return ret;
}

/**
 * Configure the operating parameters for the FIFO.
 * @param dev - The device structure.
 * @param mode - FIFO Mode. Specifies FIFO operating mode.
 *		Accepted values: ADXL357_FIFO_BYPASSED
 *				 ADXL357_FIFO_STREAMED
 *				 ADXL357_FIFO_TRIGGERED
 *				 ADXL357_FIFO_OLD_SAVED
 * @param format - FIFO Format. Specifies the data is stored in the FIFO buffer.
 *		Accepted values: ADXL357_XYZ_FIFO
 *				 ADXL357_X_FIFO
 *				 ADXL357_Y_FIFO
 *				 ADXL357_XY_FIFO
 *				 ADXL357_Z_FIFO
 *				 ADXL357_XZ_FIFO
 *				 ADXL357_YZ_FIFO
 *				 ADXL357_XYZ_PEAK_FIFO
 * @param fifo_samples - FIFO Samples. Watermark number of FIFO samples that
 *			triggers a FIFO_FULL condition when reached.
 *			Values range from 0 to 512.

 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_configure_fifo(struct device *dev,
			       u16_t fifo_samples)
{
	struct adxl357_data *data = dev->driver_data;
	u8_t fifo_config;
	int ret;

	if (fifo_samples > 512) {
		return -EINVAL;
	}

	/*
	 * All FIFO modes must be configured while in standby mode.
	 */
	ret = adxl357_set_op_mode(dev, ADXL357_STANDBY);
	if (ret) {
		return ret;
	}

	fifo_config = (ADXL357_FIFO_CTL_FORMAT_MODE(format) |
		       ADXL357_FIFO_CTL_MODE_MODE(mode) |
		       ADXL357_FIFO_CTL_SAMPLES_MODE(fifo_samples));

	ret = adxl357_reg_write(dev, ADXL357_FIFO_CTL, fifo_config);
	if (ret) {
		return ret;
	}
	ret = adxl357_reg_write(dev, ADXL357_FIFO_SAMPLES, fifo_samples & 0xFF);
	if (ret) {
		return ret;
	}

	data->fifo_config.fifo_format = format;
	data->fifo_config.fifo_mode = mode;
	data->fifo_config.fifo_samples = fifo_samples;

	return 0;
}

/**
 * Retrieve 3-axis acceleration data
 * @param dev - The device structure.
 * @param maxpeak - Retrieve the highest magnitude (x, y, z) sample recorded
 *		    since the last read of the MAXPEAK registers
 * @param accel_data - pointer to a variable of type adxl357_xyz_accel_data
 *		      where (x, y, z) acceleration data will be stored.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl357_get_accel_data(struct device *dev, bool maxpeak,
			   struct adxl357_xyz_accel_data *accel_data)
{
	u8_t buf[6];
	u8_t status1;
	int ret;

	if (!IS_ENABLED(CONFIG_ADXL357_TRIGGER)) {
		do {
			adxl357_get_status(dev, &status1, NULL, NULL);
		} while (!(ADXL357_STATUS_1_DATA_RDY(status1)));
	}

	ret = adxl357_reg_read_multiple(dev, maxpeak ? ADXL357_X_MAXPEAK_H :
					ADXL357_X_DATA_H, buf, 6);

	accel_data->x = (buf[0] << 8) | (buf[1] & 0xF0);
	accel_data->y = (buf[2] << 8) | (buf[3] & 0xF0);
	accel_data->z = (buf[4] << 8) | (buf[5] & 0xF0);

	return ret;
}

static int adxl357_attr_set_odr(struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	enum adxl357_odr odr;

	switch (val->val1) {
	case 400:
		odr = ADXL357_ODR_400HZ;
		break;
	case 800:
		odr = ADXL357_ODR_800HZ;
		break;
	case 1600:
		odr = ADXL357_ODR_1600HZ;
		break;
	case 3200:
		odr = ADXL357_ODR_3200HZ;
		break;
	case 6400:
		odr = ADXL357_ODR_6400HZ;
		break;
	default:
		return -EINVAL;
	}

	return adxl357_set_odr_lpf(dev, odr);
}

static int adxl357_attr_set_thresh(struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	const struct adxl357_dev_config *cfg = dev->config_info;
	struct adxl357_activity_threshold threshold;
	s32_t value;
	s64_t micro_ms2 = val->val1 * 1000000LL + val->val2;
	u8_t reg;

	value = abs((micro_ms2 * 10) / SENSOR_G);

	if (value > 2047) {
		return -EINVAL;
	}

	threshold.thresh = value;
	threshold.enable = cfg->activity_th.enable;
	threshold.referenced = cfg->activity_th.referenced;

	if (attr ==  SENSOR_ATTR_UPPER_THRESH) {
		reg = ADXL357_X_THRESH_ACT_H;
	} else {
		reg = ADXL357_X_THRESH_INACT_H;
	}

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		return adxl357_set_activity_threshold(dev, reg, &threshold);
	case SENSOR_CHAN_ACCEL_Y:
		return adxl357_set_activity_threshold(dev, reg + 2, &threshold);
	case SENSOR_CHAN_ACCEL_Z:
		return adxl357_set_activity_threshold(dev, reg + 4, &threshold);
	case SENSOR_CHAN_ACCEL_XYZ:
		return adxl357_set_activity_threshold_xyz(dev, reg, &threshold);
	default:
		LOG_ERR("attr_set() not supported on this channel");
		return -ENOTSUP;
	}
}

static int adxl357_attr_set(struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return adxl357_attr_set_odr(dev, chan, attr, val);
	case SENSOR_ATTR_UPPER_THRESH:
	case SENSOR_ATTR_LOWER_THRESH:
		return adxl357_attr_set_thresh(dev, chan, attr, val);
	default:
		return -ENOTSUP;
	}
}

static int adxl357_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct adxl357_data *data = dev->driver_data;
	const struct adxl357_dev_config *cfg = dev->config_info;

	return adxl357_get_accel_data(dev, cfg->max_peak_detect_mode,
				      &data->sample);
}

static void adxl357_accel_convert(struct sensor_value *val, s16_t value)
{
	/*
	 * Sensor resolution is 100mg/LSB, 12-bit value needs to be right
	 * shifted by 4 or divided by 16. Overall this results in a scale of 160
	 */
	s32_t micro_ms2 = value * (SENSOR_G / (16 * 1000 / 100));

	val->val1 = micro_ms2 / 1000000;
	val->val2 = micro_ms2 % 1000000;
}

static int adxl357_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct adxl357_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		adxl357_accel_convert(val, data->sample.x);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		adxl357_accel_convert(val, data->sample.y);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		adxl357_accel_convert(val, data->sample.z);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		adxl357_accel_convert(val++, data->sample.x);
		adxl357_accel_convert(val++, data->sample.y);
		adxl357_accel_convert(val, data->sample.z);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api adxl357_api_funcs = {
	.attr_set     = adxl357_attr_set,
	.sample_fetch = adxl357_sample_fetch,
	.channel_get  = adxl357_channel_get,
#ifdef CONFIG_ADXL357_TRIGGER
	.trigger_set = adxl357_trigger_set,
#endif

};

static int adxl357_probe(struct device *dev)
{
	const struct adxl357_dev_config *cfg = dev->config_info;
	u8_t dev_id, part_id;
	int ret;

	ret = adxl357_reg_read(dev, ADXL357_DEVID, &dev_id);
	if (ret) {
		return ret;
	}
	ret = adxl357_reg_read(dev, ADXL357_PARTID, &part_id);
	if (ret) {
		return ret;
	}

	if (dev_id != ADXL357_DEVID_VAL || part_id != ADXL357_PARTID_VAL) {
		LOG_ERR("failed to read id (0x%X:0x%X)", dev_id, part_id);
		return -ENODEV;
	}

#ifdef CONFIG_ADXL357_I2C
	/*
	 * When sharing an SDA bus, the ADXL357 Silcon REV < 3  may prevent
	 * communication with other devices on that bus.
	 */
	adxl357_reg_read(dev, ADXL357_REVID, &dev_id);
	if (dev_id < 3) {
		LOG_WRN("The ADXL357 Rev %u only supports point to point I2C communication!",
			    dev_id);
	}
#endif

	/* Device settings */
	ret = adxl357_set_op_mode(dev, ADXL357_STANDBY);
	if (ret) {
		return ret;
	}

	ret = adxl357_reset(dev);
	if (ret) {
		return ret;
	}

	ret = adxl357_set_hpf_corner(dev, cfg->hpf);
	if (ret) {
		return ret;
	}

	ret = adxl357_set_odr_lpf(dev, cfg->odr);
	if (ret) {
		return ret;
	}

	ret = adxl357_set_autosleep(dev, cfg->autosleep);
	if (ret) {
		return ret;
	}

	ret = adxl357_set_activity_threshold_xyz(dev, ADXL357_X_THRESH_ACT_H,
						 &cfg->activity_th);
	if (ret) {
		return ret;
	}

	ret = adxl357_configure_fifo(dev, cfg->fifo_config.fifo_samples);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_ADXL357_TRIGGER
	if (adxl357_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
#endif

	ret = adxl357_interrupt_config(dev, cfg->int1_config, cfg->int2_config);
	if (ret) {
		return ret;
	}

	ret = adxl357_set_op_mode(dev, cfg->op_mode);
	if (ret) {
		return ret;
	}

	return adxl357_set_act_proc_mode(dev, cfg->act_proc_mode);
}

static int adxl357_init(struct device *dev)
{
	struct adxl357_data *data = dev->driver_data;
	const struct adxl357_dev_config *cfg = dev->config_info;

#ifdef CONFIG_ADXL357_I2C
	data->bus  = device_get_binding(cfg->i2c_port);
	if (data->bus  == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			    cfg->i2c_port);
		return -EINVAL;
	}
#endif
#ifdef CONFIG_ADXL357_SPI
	data->bus = device_get_binding(cfg->spi_port);
	if (!data->bus) {
		LOG_ERR("spi device not found: %s", cfg->spi_port);
		return -EINVAL;
	}
	/* CPOL=0, CPHA=0, max 10MHz */
	data->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	data->spi_cfg.frequency = cfg->spi_max_frequency;
	data->spi_cfg.slave = cfg->spi_slave;

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	/* handle SPI CS thru GPIO if it is the case */

	data->adxl357_cs_ctrl.gpio_dev = device_get_binding(cfg->gpio_cs_port);
	if (!data->adxl357_cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	data->adxl357_cs_ctrl.gpio_pin = cfg->cs_gpio;
	data->adxl357_cs_ctrl.delay = 0U;

	data->spi_cfg.cs = &data->adxl357_cs_ctrl;
#endif
#endif /* CONFIG_ADXL357_SPI */

	if (adxl357_probe(dev) < 0) {
		return -ENODEV;
	}

	return 0;
}

static struct adxl357_data adxl357_data;

static const struct adxl357_dev_config adxl357_config = {
#ifdef CONFIG_ADXL357_I2C
	.i2c_port = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
#endif
#ifdef CONFIG_ADXL357_SPI
	.spi_port = DT_INST_BUS_LABEL(0),
	.spi_slave = DT_INST_REG_ADDR(0),
	.spi_max_frequency = DT_INST_PROP(0, spi_max_frequency),
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	.gpio_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
	.cs_gpio = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
#endif
#endif
#ifdef CONFIG_ADXL357_TRIGGER
	.gpio_port = DT_INST_GPIO_LABEL(0, int1_gpios),
	.int_gpio = DT_INST_GPIO_PIN(0, int1_gpios),
	.int_flags = DT_INST_GPIO_FLAGS(0, int1_gpios),
#endif

	.max_peak_detect_mode = IS_ENABLED(CONFIG_ADXL357_PEAK_DETECT_MODE),

#ifdef CONFIG_ADXL357_ODR_400HZ
	.odr = ADXL357_ODR_400HZ,
#elif CONFIG_ADXL357_ODR_800HZ
	.odr = ADXL357_ODR_800HZ,
#elif CONFIG_ADXL357_ODR_1600HZ
	.odr = ADXL357_ODR_1600HZ,
#elif CONFIG_ADXL357_ODR_3200HZ
	.odr = ADXL357_ODR_3200HZ,
#elif CONFIG_ADXL357_ODR_6400HZ
	.odr = ADXL357_ODR_6400HZ,
#endif

#ifdef CONFIG_ADXL357_BW_200HZ
	.bw = ADXL357_BW_200HZ,
#elif CONFIG_ADXL357_BW_400HZ
	.bw = ADXL357_BW_400HZ,
#elif CONFIG_ADXL357_BW_800HZ
	.bw = ADXL357_BW_800HZ,
#elif CONFIG_ADXL357_BW_1600HZ
	.bw = ADXL357_BW_1600HZ,
#elif CONFIG_ADXL357_BW_3200HZ
	.bw = ADXL357_BW_3200HZ,
#elif CONFIG_ADXL357_LPF_DISABLE
	.bw = ADXL357_BW_LPF_DISABLED,
#endif

#ifdef CONFIG_ADXL357_HPF_CORNER0
	.hpf = ADXL357_HPF_CORNER_0,
#elif CONFIG_ADXL357_HPF_CORNER1
	.hpf = ADXL357_HPF_CORNER_1,
#elif CONFIG_ADXL357_HPF_CORNER2
	.hpf = ADXL357_HPF_CORNER_2,
#elif CONFIG_ADXL357_HPF_CORNER3
	.hpf = ADXL357_HPF_CORNER_3,
#elif CONFIG_ADXL357_HPF_DISABLE
	.hpf = ADXL357_HPF_DISABLED,
#endif

#ifdef CONFIG_ADXL357_TRIGGER
	.act_proc_mode = ADXL357_LINKED,
#else
	.act_proc_mode = ADXL357_LOOPED,
#endif
	.th_mode = ADXL357_INSTANT_ON_LOW_TH,
	.autosleep = false,
	.wur = ADXL357_WUR_52ms,

	.activity_th.thresh = CONFIG_ADXL357_ACTIVITY_THRESHOLD / 100,
	.activity_th.referenced =
		IS_ENABLED(CONFIG_ADXL357_REFERENCED_ACTIVITY_DETECTION_MODE),
	.activity_th.enable = 1,
	.activity_time = CONFIG_ADXL357_ACTIVITY_TIME,

	.inactivity_th.thresh = CONFIG_ADXL357_INACTIVITY_THRESHOLD / 100,
	.inactivity_th.referenced =
		IS_ENABLED(CONFIG_ADXL357_REFERENCED_ACTIVITY_DETECTION_MODE),
	.inactivity_th.enable = 1,
	.inactivity_time = CONFIG_ADXL357_INACTIVITY_TIME,

	.filter_settle = ADXL357_FILTER_SETTLE_370,
	.fifo_config.fifo_mode = ADXL357_FIFO_STREAMED,
	.fifo_config.fifo_format = ADXL357_XYZ_PEAK_FIFO,
	.fifo_config.fifo_samples = 128,

	.op_mode = ADXL357_FULL_BW_MEASUREMENT,
};

DEVICE_AND_API_INIT(adxl357, DT_INST_LABEL(0), adxl357_init,
		    &adxl357_data, &adxl357_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &adxl357_api_funcs);

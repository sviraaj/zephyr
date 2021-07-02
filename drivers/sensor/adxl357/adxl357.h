/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADXL357_ADXL357_H_
#define ZEPHYR_DRIVERS_SENSOR_ADXL357_ADXL357_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/i2c.h>
#include <sys/util.h>

/*
 * ADXL357 registers definition
 */
#define ADXL357_DEVID		0x00u  /* Analog Devices accelerometer ID */
#define ADXL357_DEVID_MST	0x01u  /* Analog Devices MEMS device ID */
#define ADXL357_PARTID		0x02u  /* Device ID */
#define ADXL357_REVID		0x03u  /* product revision ID */
#define ADXL357_STATUS	    0x04u  /* Status register */
#define ADXL357_FIFO_ENTRIES	0x05u  /* Valid data samples in the FIFO */
#define ADXL357_TEMP2	    0x06u  /* Status register */
#define ADXL357_TEMP1	    0x07u  /* Status register */
#define ADXL357_X_DATA3	    0x08u  /* X-axis acceleration data [11:4] */
#define ADXL357_X_DATA2 	0x09u  /* X-axis acceleration data [3:0] */
#define ADXL357_X_DATA1 	0x0Au  /* X-axis acceleration data [3:0] */
#define ADXL357_Y_DATA3 	0x0Bu  /* Y-axis acceleration data [11:4] */
#define ADXL357_Y_DATA2 	0x0Cu  /* Y-axis acceleration data [3:0] */
#define ADXL357_Y_DATA1 	0x0Du  /* Y-axis acceleration data [3:0] */
#define ADXL357_Z_DATA3 	0x0Eu  /* Z-axis acceleration data [11:4] */
#define ADXL357_Z_DATA2 	0x0Fu  /* Z-axis acceleration data [3:0] */
#define ADXL357_Z_DATA1 	0x10u  /* Z-axis acceleration data [3:0] */
#define ADXL357_FIFO_DATA 	0x11u  /* Z-axis acceleration data [3:0] */
#define ADXL357_OFFSET_X_H	0x1Eu  /* X axis offset */
#define ADXL357_OFFSET_X_L	0x1Fu  /* X axis offset */
#define ADXL357_OFFSET_Y_H	0x20u  /* Y axis offset */
#define ADXL357_OFFSET_Y_L	0x21u  /* Y axis offset */
#define ADXL357_OFFSET_Z_H	0x22u  /* Z axis offset */
#define ADXL357_OFFSET_Z_L	0x23u  /* Z axis offset */
#define ADXL357_ACT_EN  	0x24u  /* X axis Activity Threshold [15:8] */
#define ADXL357_ACT_THRESH_H	0x25u  /* X axis Activity Threshold [15:8] */
#define ADXL357_ACT_THRESH_L	0x26u  /* X axis Activity Threshold [7:0] */
#define ADXL357_ACT_COUNT   	0x27u  /* X axis Activity Threshold [7:0] */
#define ADXL357_FILTER      	0x28u  /* Activity Time */
#define ADXL357_FIFO_SAMPLES	0x29u  /* FIFO Samples */
#define ADXL357_INT_MAP	        0x2Au  /* Interrupt 1 mapping control */
#define ADXL357_SYNC	        0x2Bu  /* SYNC */
#define ADXL357_RANGE		0x2Cu  /* Timing */
#define ADXL357_POWER_CTL	0x2Du  /* Power control */
#define ADXL357_SELF_TEST	0x2Eu  /* Self Test */
#define ADXL357_RESET		0x2Fu  /* Reset */

#define ADXL357_DEVID_VAL	0xADu  /* Analog Devices accelerometer ID */
#define ADXL357_MST_DEVID_VAL	0x1Du  /* Analog Devices MEMS device ID */
#define ADXL357_PARTID_VAL	0xEDu  /* Device ID */
#define ADXL357_REVID_VAL	0x01u  /* product revision ID*/
#define ADXL357_RESET_CODE	0x52u  /* Writing code 0x52 resets the device */

#define ADXL357_READ		0x01u
#define ADXL357_REG_READ(x)	(((x & 0xFF) << 1) | ADXL357_READ)
#define ADXL357_REG_WRITE(x)	((x & 0xFF) << 1)
#define ADXL357_TO_I2C_REG(x)	((x) >> 1)

/* ADXL357_POWER_CTL */
#define ADXL357_POWER_DRDY_OFF_MSK	    BIT(2)
#define ADXL357_POWER_DRDY_OFF_MODE(x)	(((x) & 0x1) << 2)
#define ADXL357_POWER_TEMP_OFF_MSK	    BIT(1)
#define ADXL357_POWER_TEMP_OFF_MODE(x)	(((x) & 0x1) << 1)
#define ADXL357_POWER_CTL_MODE_MSK		BIT(0)
#define ADXL357_POWER_CTL_MODE(x)		(((x) & 0x1) << 0)

/* ADXL357_FILTER */
#define ADXL357_FILTER_HPF_CORNER_MSK			GENMASK(6, 4)
#define ADXL357_FILTER_HPF_CORNER_MODE(x)		(((x) & 0x7) << 4)
#define ADXL357_FILTER_ODR_LPF_MSK			    GENMASK(3, 0)
#define ADXL357_FILTER_ODR_LPF_MODE(x)		    (((x) & 0xF) << 0)

/* ADXL357_STATUS */
#define ADXL357_STATUS_DATA_RDY(x)		(((x) >> 0) & 0x1)
#define ADXL357_STATUS_FIFO_FULL(x)		(((x) >> 1) & 0x1)
#define ADXL357_STATUS_FIFO_OVR(x)		(((x) >> 2) & 0x1)
#define ADXL357_STATUS_ACTIVITY(x)	    (((x) >> 3) & 0x1)
#define ADXL357_STATUS_USR_NVM_BUSY(x)	(((x) >> 4) & 0x1)

/* ADXL357_INT_MAP */
#define ADXL357_INT1_MAP_RDY_EN1_MSK		BIT(0)
#define ADXL357_INT1_MAP_RDY_EN1_MODE(x)	(((x) & 0x1) << 0)
#define ADXL357_INT1_MAP_FULL_EN1_MSK		BIT(1)
#define ADXL357_INT1_MAP_FULL_EN1_MODE(x)	(((x) & 0x1) << 1)
#define ADXL357_INT1_MAP_OVR_EN1_MSK		BIT(2)
#define ADXL357_INT1_MAP_OVR_EN1_MODE(x)	(((x) & 0x1) << 2)
#define ADXL357_INT1_MAP_ACT_EN1_MSK		BIT(3)
#define ADXL357_INT1_MAP_ACT_EN1_MODE(x)	(((x) & 0x1) << 3)
#define ADXL357_INT1_MAP_RDY_EN2_MSK		BIT(4)
#define ADXL357_INT1_MAP_RDY_EN2_MODE(x)	(((x) & 0x1) << 4)
#define ADXL357_INT1_MAP_FULL_EN2_MSK		BIT(5)
#define ADXL357_INT1_MAP_FULL_EN2_MODE(x)	(((x) & 0x1) << 5)
#define ADXL357_INT1_MAP_OVR_EN2_MSK		BIT(6)
#define ADXL357_INT1_MAP_OVR_EN2_MODE(x)	(((x) & 0x1) << 6)
#define ADXL357_INT1_MAP_ACT_EN2_MSK		BIT(7)
#define ADXL357_INT1_MAP_ACT_EN2_MODE(x)	(((x) & 0x1) << 7)

/* ADXL357 RANGE */
#define ADXL357_RANGE_RANGE_MSK		    GENMASK(1, 0)
#define ADXL357_RANGE_RANGE_MODE(x)		(((x) & 0x3) << 0)

/* ADXL357 ACT_EN */
#define ADXL357_ACT_EN_X_MSK		    BIT(0)
#define ADXL357_ACT_EN_X_MODE(x)		(((x) & 0x1) << 0)
#define ADXL357_ACT_EN_Y_MSK		    BIT(1)
#define ADXL357_ACT_EN_Y_MODE(x)		(((x) & 0x1) << 1)
#define ADXL357_ACT_EN_Z_MSK		    BIT(2)
#define ADXL357_ACT_EN_Z_MODE(x)		(((x) & 0x1) << 2)

enum adxl357_axis {
	ADXL357_X_AXIS,
	ADXL357_Y_AXIS,
	ADXL357_Z_AXIS
};

enum adxl357_op_mode {
	ADXL357_STANDBY,
	ADXL357_WAKE_UP,
	ADXL357_INSTANT_ON,
	ADXL357_FULL_BW_MEASUREMENT
};

enum adxl357_range {
    ADXL357_RANGE_10G = 1,
    ADXL357_RANGE_20G,
    ADXL357_RANGE_40G,
};

enum adxl357_odr_lpf {
	ADXL357_ODR_4000HZ,
	ADXL357_ODR_2000HZ,
	ADXL357_ODR_1000HZ,
	ADXL357_ODR_500HZ,
	ADXL357_ODR_250HZ,
	ADXL357_ODR_125HZ,
	ADXL357_ODR_62HZ,
	ADXL357_ODR_31HZ,
	ADXL357_ODR_15HZ,
	ADXL357_ODR_7HZ,
	ADXL357_ODR_3HZ,
};

enum adxl357_hpf_corner {
	ADXL357_HPF_DISABLED,
	ADXL357_HPF_CORNER_0,
	ADXL357_HPF_CORNER_1,
	ADXL357_HPF_CORNER_2,
	ADXL357_HPF_CORNER_3,
	ADXL357_HPF_CORNER_4,
	ADXL357_HPF_CORNER_5,
};

struct adxl357_fifo_config {
	u16_t fifo_samples;
};

struct adxl357_activity_threshold {
	u16_t thresh;
	bool referenced;
	bool enable;
};

struct adxl357_xyz_accel_data {
	s32_t x;
	s32_t y;
	s32_t z;
};

struct adxl357_data {
	struct device *bus;
#ifdef CONFIG_ADXL357_SPI
	struct spi_config spi_cfg;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	struct spi_cs_control adxl357_cs_ctrl;
#endif
#endif
	struct adxl357_xyz_accel_data sample;
	struct adxl357_fifo_config fifo_config;

#ifdef CONFIG_ADXL357_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t th_handler;
	struct sensor_trigger th_trigger;
	sensor_trigger_handler_t drdy_handler;
	struct sensor_trigger drdy_trigger;

#if defined(CONFIG_ADXL357_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_ADXL357_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_ADXL357_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif
#endif /* CONFIG_ADXL357_TRIGGER */
};

struct adxl357_dev_config {
#ifdef CONFIG_ADXL357_I2C
	const char *i2c_port;
	u16_t i2c_addr;
#endif
#ifdef CONFIG_ADXL357_SPI
	const char *spi_port;
	u16_t spi_slave;
	u32_t spi_max_frequency;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	const char *gpio_cs_port;
	gpio_pin_t cs_gpio;
#endif
#endif /* CONFIG_ADXL357_SPI */
#ifdef CONFIG_ADXL357_TRIGGER
	const char *gpio_port;
	gpio_pin_t int_gpio;
	gpio_dt_flags_t int_flags;
#endif
	bool max_peak_detect_mode;

	/* Device Settings */
	bool autosleep;

	struct adxl357_activity_threshold activity_th;
	struct adxl357_activity_threshold inactivity_th;
	struct adxl357_fifo_config fifo_config;

	enum adxl357_odr_lpf odr_lpf;
	enum adxl357_hpf_corner hpf;
	enum adxl357_op_mode op_mode;
    enum adxl357_range range;

	u16_t inactivity_time;
	u8_t activity_time;
	u8_t int1_config;
	u8_t int2_config;
};

#ifdef CONFIG_ADXL357_TRIGGER
int adxl357_get_status(struct device *dev,
		       u8_t *status1, u8_t *status2, u16_t *fifo_entries);

int adxl357_reg_write_mask(struct device *dev,
			   u8_t reg_addr, u32_t mask, u8_t data);

int adxl357_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int adxl357_init_interrupt(struct device *dev);
#endif /* CONFIG_ADXL357_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_ADXL357_ADXL357_H_ */

/*
 * Copyright (c) 2018 Aapo Vienamo
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include "gpio_utils.h"

/** Cache of the output configuration and data of the pins */
struct gpio_sx1509b_pin_state {
	u16_t input_disable;
	u16_t pull_up;
	u16_t pull_down;
	u16_t open_drain;
	u16_t polarity;
	u16_t dir;
	u16_t data;
	u16_t interrupt;
    u16_t debounce_en;
	u32_t interrupt_sense;
};

/** Runtime driver data */
struct gpio_sx1509b_drv_data {
	struct device *i2c_master;
	struct device *gpio_int;
	struct gpio_sx1509b_pin_state pin_state;
	struct gpio_sx1509b_config* cfg;
	struct gpio_callback gpio_cb;
	u8_t interrupt_sup;
	struct k_sem lock;

	struct k_work work;
	struct device *dev;

	/* Enabled INT pins generating a cb */
	u32_t cb_pins;
	/* user ISR cb */
	sys_slist_t cb;
};

/** Configuration data */
struct gpio_sx1509b_config {
	const char *i2c_master_dev_name;
	const char *gpio_int_dev_name;
	u16_t i2c_slave_addr;
	u16_t gpio_pin;
	u16_t gpio_flags;
};

/* General configuration register addresses */
enum {
	/* TODO: Add rest of the regs */
	SX1509B_REG_CLOCK               = 0x1e,
	SX1509B_REG_RESET               = 0x7d,
};

/* Magic values for softreset */
enum {
	SX1509B_REG_RESET_MAGIC0	= 0x12,
	SX1509B_REG_RESET_MAGIC1	= 0x34,
};

/* Register bits for SX1509B_REG_CLOCK */
enum {
	SX1509B_REG_CLOCK_FOSC_OFF	= 0 << 5,
	SX1509B_REG_CLOCK_FOSC_EXT	= 1 << 5,
	SX1509B_REG_CLOCK_FOSC_INT_2MHZ	= 2 << 5,
};

/* Pin configuration register addresses */
enum {
	SX1509B_REG_INPUT_DISABLE	= 0x00,
	SX1509B_REG_PULL_UP		= 0x06,
	SX1509B_REG_PULL_DOWN		= 0x08,
	SX1509B_REG_OPEN_DRAIN		= 0x0a,
	SX1509B_REG_DIR			= 0x0e,
	SX1509B_REG_DATA		= 0x10,
	SX1509B_REG_LED_DRIVER_ENABLE	= 0x20,
    SX1509B_REG_DEBOUNCE_CONFIG = 0x22,
	SX1509B_REG_DEBOUNCE_ENABLE	= 0x23,
};

/* Interrupt config register addresses */
enum {
	SX1509B_REG_INTERRUPT_MASK   = 0x12,
	SX1509B_REG_SENSE_B          = 0x14,
	SX1509B_REG_SENSE_A          = 0x16,
	SX1509B_REG_INTERRUPT_SOURCE = 0x18,
};

/* Edge sensitivity types */
enum {
	SX1509B_EDGE_NONE     = 0x00,
	SX1509B_EDGE_RISING   = 0x01,
	SX1509B_EDGE_FALLING  = 0x02,
	SX1509B_EDGE_BOTH     = 0x03,
};

/**
 * @brief Write a big-endian word to an internal address of an I2C slave.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dev_addr Address of the I2C device for writing.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i2c_reg_write_word_be(struct device *dev, u16_t dev_addr,
					u8_t reg_addr, u16_t value)
{
	u8_t tx_buf[3] = { reg_addr, value >> 8, value & 0xff };

	return i2c_write(dev, tx_buf, 3, dev_addr);
}

/**
 * @brief Write a big-endian byte to an internal address of an I2C slave.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dev_addr Address of the I2C device for writing.
 * @param reg_addr Address of the internal register being written.
 * @param value Value to be written to internal register.
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
static inline int i2c_reg_write_byte_be(struct device *dev, u16_t dev_addr,
					u8_t reg_addr, u8_t value)
{
	u8_t tx_buf[3] = { reg_addr, value };

	return i2c_write(dev, tx_buf, 2, dev_addr);
}

static int sx1509b_handle_interrupt(void* arg)
{
	struct device* dev = (struct device *) arg;
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	int ret = 0;
	u16_t int_source;

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret = i2c_burst_read(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_INTERRUPT_SOURCE, (u8_t *)&int_source,
				 sizeof(int_source));
	if (ret) {
		goto out;
	}

	int_source = sys_be16_to_cpu(int_source);

	if ((int_source & drv_data->cb_pins) != 0) {
		gpio_fire_callbacks(&drv_data->cb, dev, int_source);
	}

    /* reset interrupts */
	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_INTERRUPT_SOURCE, 0xffff);
	if (ret) {
		goto out;
	}

out:
	k_sem_give(&drv_data->lock);

	return ret;
}

static void sx1509b_work_handler(struct k_work *work)
{
	struct gpio_sx1509b_drv_data *drv_data =
		CONTAINER_OF(work, struct gpio_sx1509b_drv_data, work);

	sx1509b_handle_interrupt(drv_data->dev);
}

static void sx1509_int_cb(struct device *dev, struct gpio_callback *gpio_cb,
			   u32_t pins)
{
	struct gpio_sx1509b_drv_data *drv_data = CONTAINER_OF(gpio_cb,
		struct gpio_sx1509b_drv_data, gpio_cb);

	ARG_UNUSED(pins);

	k_work_submit(&drv_data->work);
}

/**
 * @brief Configure pin or port
 *
 * @param dev Device struct of the SX1509B
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_sx1509b_config(struct device *dev, int access_op, u32_t pin,
				   int flags)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	struct gpio_sx1509b_pin_state *pins = &drv_data->pin_state;
	int ret = 0;

	if ((flags & GPIO_INT) && (drv_data->interrupt_sup == 0)) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			pins->dir |= BIT(pin);
			pins->input_disable &= ~BIT(pin);
		} else {
			pins->dir &= ~BIT(pin);
			pins->input_disable |= BIT(pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			pins->pull_up |= BIT(pin);
		} else {
			pins->pull_up &= ~BIT(pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			pins->pull_down |= BIT(pin);
		} else {
			pins->pull_down &= ~BIT(pin);
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
			pins->pull_up &= ~BIT(pin);
			pins->pull_down &= ~BIT(pin);
		}
		if (flags & GPIO_DS_DISCONNECT_HIGH) {
			pins->open_drain |= BIT(pin);
		} else {
			pins->open_drain &= ~BIT(pin);
		}
		if (flags & GPIO_POL_INV) {
			pins->polarity |= BIT(pin);
		} else {
			pins->polarity &= ~BIT(pin);
		}
		if (flags & GPIO_INT) {
			pins->interrupt &= ~BIT(pin);
			if (flags & GPIO_INT_DOUBLE_EDGE)
			{
				pins->interrupt_sense |= (SX1509B_EDGE_BOTH << (pin * 2));
			}
			else if ((flags & GPIO_INT_ACTIVE_LOW) == 0)
			{
				pins->interrupt_sense |= (SX1509B_EDGE_FALLING << (pin * 2));
			}
			else if (flags & GPIO_INT_ACTIVE_HIGH)
			{
				pins->interrupt_sense |= (SX1509B_EDGE_RISING << (pin * 2));
			}
		} else {
			pins->interrupt |= BIT(pin);
		}
		if (flags & GPIO_INT_DEBOUNCE) {
			pins->debounce_en |= BIT(pin);
		}
		else {
			pins->debounce_en &= ~BIT(pin);
		}
		break;
	case GPIO_ACCESS_BY_PORT:
		if ((flags & GPIO_DIR_MASK) == GPIO_DIR_IN) {
			pins->dir = 0xffff;
			pins->input_disable = 0x0000;
		} else {
			pins->dir = 0x0000;
			pins->input_disable = 0xffff;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
			pins->pull_up = 0xffff;
		} else {
			pins->pull_up = 0x0000;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
			pins->pull_down = 0xffff;
		} else {
			pins->pull_down = 0x0000;
		}
		if ((flags & GPIO_PUD_MASK) == GPIO_PUD_NORMAL) {
			pins->pull_up = 0x0000;
			pins->pull_down = 0x0000;
		}
		if (flags & GPIO_DS_DISCONNECT_HIGH) {
			pins->open_drain = 0xffff;
		} else {
			pins->open_drain = 0x0000;
		}
		if (flags & GPIO_POL_INV) {
			pins->polarity = 0xffff;
		} else {
			pins->polarity = 0x0000;
		}
		if (flags & GPIO_INT) {
			pins->interrupt = 0x0000;
			if (flags & GPIO_INT_DOUBLE_EDGE)
			{
				pins->interrupt_sense = 0xffffffff;
			}
			else if (flags & GPIO_INT_ACTIVE_LOW)
			{
				pins->interrupt_sense = 0xaaaaaaaa;
			}
			else if (flags & GPIO_INT_ACTIVE_HIGH)
			{
				pins->interrupt_sense = 0x55555555;
			}
		} else {
			pins->interrupt = 0xffff;
		}
		if (flags & GPIO_INT_DEBOUNCE) {
			pins->debounce_en = 0xffff;
		}
		else {
			pins->debounce_en = 0x0000;
		}
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_DIR, pins->dir);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_INPUT_DISABLE,
					pins->input_disable);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_PULL_UP,
					pins->pull_up);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_PULL_DOWN,
					pins->pull_down);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_OPEN_DRAIN,
					pins->open_drain);
	if (ret) {
		goto out;
	}

	/* FIXME Need to write only one byte */
	ret = i2c_reg_write_byte_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_DEBOUNCE_CONFIG,
					CONFIG_GPIO_SX1509B_DEBOUNCE_TIME);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_DEBOUNCE_ENABLE,
					pins->debounce_en);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_INTERRUPT_MASK,
					pins->interrupt);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_SENSE_B,
					(pins->interrupt_sense >> 16) & 0xffff);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_SENSE_A,
					pins->interrupt_sense & 0xffff);

out:
	k_sem_give(&drv_data->lock);
	return ret;
}

/**
 * @brief Set the pin or port output
 *
 * @param dev Device struct of the SX1509B
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_sx1509b_write(struct device *dev, int access_op, u32_t pin,
				  u32_t value)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	u16_t *pin_data = &drv_data->pin_state.data;
	int ret = 0;

	k_sem_take(&drv_data->lock, K_FOREVER);

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		if (value) {
			*pin_data |= BIT(pin);
		} else {
			*pin_data &= ~BIT(pin);
		}
		break;
	case GPIO_ACCESS_BY_PORT:
		*pin_data = value;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	ret = i2c_reg_write_word_be(drv_data->i2c_master, cfg->i2c_slave_addr,
					SX1509B_REG_DATA, *pin_data);
out:
	k_sem_give(&drv_data->lock);
	return ret;
}

/**
 * @brief Read the pin or port data
 *
 * @param dev Device struct of the SX1509B
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_sx1509b_read(struct device *dev, int access_op, u32_t pin,
				  u32_t *value)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	u16_t pin_data;
	int ret;

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret = i2c_burst_read(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_DATA, (u8_t *)&pin_data,
				 sizeof(pin_data));
	if (ret) {
		goto out;
	}

	pin_data = sys_be16_to_cpu(pin_data);

	switch (access_op) {
	case GPIO_ACCESS_BY_PIN:
		*value = !!(pin_data & (BIT(pin)));
		break;
	case GPIO_ACCESS_BY_PORT:
		*value = pin_data;
		break;
	default:
		ret = -ENOTSUP;
	}

out:
	k_sem_give(&drv_data->lock);
	return ret;
}

/**
 * @brief Initialization function of SX1509B
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_sx1509b_init(struct device *dev)
{
	const struct gpio_sx1509b_config *cfg = dev->config->config_info;
	struct gpio_sx1509b_drv_data *drv_data = dev->driver_data;
	int ret;

	/* storing dev ptr with drv data to be used in gpio_cb in interrupt ctx */
	drv_data->dev = dev;

	drv_data->i2c_master = device_get_binding(cfg->i2c_master_dev_name);
	if (!drv_data->i2c_master) {
		ret = -EINVAL;
		goto out;
	}

#ifdef DT_INST_0_SEMTECH_SX1509B_INT1_GPIOS_CONTROLLER
	drv_data->gpio_int = device_get_binding(cfg->gpio_int_dev_name);
	if (!drv_data->gpio_int) {
		/* interrupt not supported */
		drv_data->interrupt_sup = 0;
	}
	else
	{
		drv_data->interrupt_sup = 1;

		drv_data->work.handler = sx1509b_work_handler;

		gpio_pin_configure(drv_data->gpio_int, cfg->gpio_pin,
				(GPIO_DIR_IN | GPIO_PUD_PULL_UP | GPIO_INT | GPIO_INT_EDGE
				 | GPIO_INT_ACTIVE_LOW));

		gpio_init_callback(&drv_data->gpio_cb, sx1509_int_cb, BIT(cfg->gpio_pin));
		gpio_add_callback(drv_data->gpio_int, &drv_data->gpio_cb);
		gpio_pin_enable_callback(drv_data->gpio_int, cfg->gpio_pin);
	}
#elif
	/* interrupt not supported */
	drv_data->interrupt_sup = 0;
#endif

	/* Reset state */
	drv_data->pin_state = (struct gpio_sx1509b_pin_state) {
		.input_disable	 = 0x0000,
		.pull_up		 = 0x0000,
		.pull_down		 = 0x0000,
		.open_drain		 = 0x0000,
		.dir			 = 0xffff,
		.data			 = 0xffff,
		.interrupt		 = 0xffff,
		.interrupt_sense = 0x00000000,
	};

	ret = i2c_reg_write_byte(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_RESET, SX1509B_REG_RESET_MAGIC0);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_byte(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_RESET, SX1509B_REG_RESET_MAGIC1);
	if (ret) {
		goto out;
	}

	ret = i2c_reg_write_byte(drv_data->i2c_master, cfg->i2c_slave_addr,
				 SX1509B_REG_CLOCK,
				 SX1509B_REG_CLOCK_FOSC_INT_2MHZ);
	if (ret) {
		goto out;
	}

out:
	k_sem_give(&drv_data->lock);
	return ret;
}

static int gpio_sx1509b_manage_callback(struct device *dev,
					  struct gpio_callback *callback,
					  bool set)
{
	struct gpio_sx1509b_drv_data *data = dev->driver_data;

	return gpio_manage_callback(&data->cb, callback, set);
}

static int gpio_sx1509b_enable_callback(struct device *dev,
					  int access_op, u32_t pin)
{
	struct gpio_sx1509b_drv_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins |= BIT(pin);

	return 0;
}

static int gpio_sx1509b_disable_callback(struct device *dev,
					   int access_op, u32_t pin)
{
	struct gpio_sx1509b_drv_data *data = dev->driver_data;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	data->cb_pins &= ~BIT(pin);

	return 0;
}

static const struct gpio_sx1509b_config gpio_sx1509b_cfg = {
	.i2c_master_dev_name = DT_INST_0_SEMTECH_SX1509B_BUS_NAME,
	.i2c_slave_addr	= DT_INST_0_SEMTECH_SX1509B_BASE_ADDRESS,

#ifdef DT_INST_0_SEMTECH_SX1509B_INT1_GPIOS_CONTROLLER
    .gpio_int_dev_name = DT_INST_0_SEMTECH_SX1509B_INT1_GPIOS_CONTROLLER,
    .gpio_pin = DT_INST_0_SEMTECH_SX1509B_INT1_GPIOS_PIN,
    .gpio_flags = DT_INST_0_SEMTECH_SX1509B_INT1_GPIOS_FLAGS,
#endif
};

static struct gpio_sx1509b_drv_data gpio_sx1509b_drvdata = {
	.lock = Z_SEM_INITIALIZER(gpio_sx1509b_drvdata.lock, 1, 1),
};

static const struct gpio_driver_api gpio_sx1509b_drv_api_funcs = {
	.config	= gpio_sx1509b_config,
	.write	= gpio_sx1509b_write,
	.read	= gpio_sx1509b_read,

#ifdef DT_INST_0_SEMTECH_SX1509B_INT1_GPIOS_CONTROLLER
	.manage_callback = gpio_sx1509b_manage_callback,
	.enable_callback = gpio_sx1509b_enable_callback,
	.disable_callback = gpio_sx1509b_disable_callback,
#endif
};

DEVICE_AND_API_INIT(gpio_sx1509b, DT_INST_0_SEMTECH_SX1509B_LABEL,
			gpio_sx1509b_init, &gpio_sx1509b_drvdata, &gpio_sx1509b_cfg,
			POST_KERNEL, CONFIG_GPIO_SX1509B_INIT_PRIORITY,
			&gpio_sx1509b_drv_api_funcs);

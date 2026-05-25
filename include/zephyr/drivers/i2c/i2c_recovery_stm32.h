#ifndef I2C_RECOVERY_STM32_H
#define I2C_RECOVERY_STM32_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stm32_ll_i2c.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>


void i2c_recovery_stm32(I2C_TypeDef *i2c, 
                                 GPIO_TypeDef *port,
                                 uint32_t scl_pin, 
                                 uint32_t sda_pin,
                                 const struct gpio_dt_spec *scl_dt,
                                 const struct gpio_dt_spec *sda_dt);
#endif

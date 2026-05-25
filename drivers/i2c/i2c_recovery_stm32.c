
#include <soc.h>
#include <stm32_ll_i2c.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c/i2c_recovery_stm32.h>


void i2c_recovery_stm32(I2C_TypeDef *i2c, 
                                 GPIO_TypeDef *port,
                                 uint32_t scl_pin, 
                                 uint32_t sda_pin,
                                 const struct gpio_dt_spec *scl_dt,
                                 const struct gpio_dt_spec *sda_dt)
{
    // --- STEP 0: CAPTURE STATE ---
    uint32_t timing_backup = i2c->TIMINGR;
    uint32_t cr1_backup = i2c->CR1;

    if (timing_backup == 0) return;

    // --- STEP 1: PERIPHERAL SHUTDOWN & RCC RESET ---
    LL_I2C_Disable(i2c);
    k_busy_wait(100);

    /* SMART RCC RESET: Identify which I2C we are resetting */
    if (i2c == I2C1) {
        LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
        k_busy_wait(500);
        LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C1);
    } 
    else if (i2c == I2C2) {
        LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C2);
        k_busy_wait(500);
        LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_I2C2);
    }
    k_busy_wait(500);

    // --- STEP 2: GPIO TAKEOVER ---
    LL_GPIO_SetPinMode(port, scl_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(port, sda_pin, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(port, scl_pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinOutputType(port, sda_pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(port, scl_pin, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(port, sda_pin, LL_GPIO_PULL_UP);
    k_busy_wait(100);

    // --- STEP 3 & 4: PULSE SWEEP ---
    for (int i = 0; i < 80; i++) {
        gpio_pin_set_dt(scl_dt, 0);
        k_busy_wait(20);
        gpio_pin_set_dt(scl_dt, 1);
        k_busy_wait(20);
    }

    // --- STEP 5: MANUAL STOP ---
    gpio_pin_set_dt(scl_dt, 0); k_busy_wait(10);
    gpio_pin_set_dt(sda_dt, 0); k_busy_wait(10);
    gpio_pin_set_dt(scl_dt, 1); k_busy_wait(10);
    gpio_pin_set_dt(sda_dt, 1); k_busy_wait(10);

    // --- STEP 6: RESTORE MUX & ENGINE ---
    LL_GPIO_SetAFPin_0_7(port, sda_pin, LL_GPIO_AF_4);
    LL_GPIO_SetAFPin_0_7(port, scl_pin, LL_GPIO_AF_4);
    LL_GPIO_SetPinMode(port, sda_pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(port, scl_pin, LL_GPIO_MODE_ALTERNATE);

    i2c->TIMINGR = timing_backup; 
    i2c->CR1 = (cr1_backup & ~I2C_CR1_PE); 
    k_busy_wait(100);
    
    LL_I2C_Enable(i2c);
    i2c->ICR = 0xFFFFFFFF;
}

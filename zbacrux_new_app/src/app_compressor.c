/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <string.h>
#include "app_compressor.h"
#include "dev_data.h"

#define COMP_RX_BUFFER_SIZE        32U
#define MAX_DELTA_DEG_C            4U
#define MAX_NEGATIVE_TEMP          30

struct compressor_data
{
    struct device* compressor_dev;
    struct device* gpio_dev;

    struct compressor_rx_data c_rx_data;
    u16_t set_speed;
    int32_t current_temp; /* deg C * 100 */
    int32_t target_temp; /* deg C * 100 */
};

static struct k_sem rx_sem;
static struct compressor_data c_data;

/* RX thread structures */
K_THREAD_STACK_DEFINE(com_rx_stack, 1024);
struct k_thread com_rx_thread;

static u8_t c_rx_buf[COMP_RX_BUFFER_SIZE];
static u16_t c_rx_wr_idx = 0;
static u16_t c_rx_rd_idx = 0;
static u16_t c_rx_cnt = 0;

static u8_t bldc_host_fmt[16] = {
    0xAA, 0x00, 0x00, /* 0：Power off 1：Power on */
    0x00, /* Low speed setting */
    0x00, /* High speed setting */
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, /* checksum */
    0x55
};

static inline void rx_buf_put(u16_t sz)
{
    c_rx_wr_idx += sz;
    if (c_rx_wr_idx >= COMP_RX_BUFFER_SIZE)
    {
        c_rx_wr_idx = c_rx_wr_idx % COMP_RX_BUFFER_SIZE;
    }

    c_rx_cnt += sz;
}

static inline void rx_buf_get(u16_t sz)
{
    c_rx_rd_idx += sz;
    if (c_rx_rd_idx >= COMP_RX_BUFFER_SIZE)
    {
        c_rx_rd_idx = c_rx_rd_idx % COMP_RX_BUFFER_SIZE;
    }

    c_rx_cnt -= sz;
}

/**
 * @brief  External Communication interrupt handler.
 *
 * @note   Fills interfaces ring buffer with received data.
 *         When ring buffer is full the data is discarded.
 *
 * @param  *cmpsr_data->compressor_dev: uart device.
 *
 * @retval None.
 */
static void com_uart_isr(struct device* compressor_dev)
{
    int rx = 0, ret;

    /* get all of the data off UART as fast as we can */
    while (uart_irq_update(compressor_dev)
            && uart_irq_is_pending(compressor_dev)
            && uart_irq_rx_ready(compressor_dev)) {
        ret = uart_fifo_read(compressor_dev, c_rx_buf + c_rx_wr_idx,
                sizeof(c_rx_buf) - c_rx_cnt);
        if (ret > 0)
        {
             rx_buf_put(ret);
        }
    }

    k_sem_give(&rx_sem);
}

static int com_uart_write(struct device *compressor_dev, const u8_t *buf,
              size_t size)
{
    if (size == 0) {
        return 0;
    }

    do {
        uart_poll_out(compressor_dev, *buf++);
    } while (--size);

    return 0;
}


static void bldc_power(u8_t on)
{
    bldc_host_fmt[2] = on;
}

static void bldc_calc_checksum()
{
    u8_t cksm = 0;

    for (int i = 0; i < 14; i++) {
        cksm += bldc_host_fmt[i];
    }

    bldc_host_fmt[14] = cksm;
}

static void set_rpm(u16_t rpm)
{
    if (rpm < MIN_RPM || rpm > MAX_RPM) {
        printk("Invalid rpm value");
    }

    bldc_host_fmt[3] = rpm & 0xFF;
    bldc_host_fmt[4] = (rpm >> 8) & 0xFF;
}

#if 0
static u16_t get_rpm()
{
    return ((bldc_host_fmt[4] << 8) | bldc_host_fmt[3]);
}
#endif

static int bldc_start(u16_t rpm)
{
    int ret;

    bldc_power(1);
    set_rpm(rpm);
    bldc_calc_checksum();

    ret = com_uart_write(c_data.compressor_dev, bldc_host_fmt, 16);

    return ret;
}

static int bldc_stop()
{
    int ret;

    bldc_power(0);
    bldc_calc_checksum();

    ret = com_uart_write(c_data.compressor_dev, bldc_host_fmt, 16);

    return ret;
}

/**
 * @brief  Drains UART.
 *
 * @note   Discards remaining data.
 *
 * @param  *iface: modem interface.
 *
 * @retval None.
 */
static void com_uart_flush(struct device* compressor_dev)
{
    u8_t c;

    while (uart_fifo_read(compressor_dev, &c, 1) > 0) {
        continue;
    }
}

static void pid_run()
{
    float cur_temp = c_data.current_temp / 100;
    float tgt_temp = c_data.target_temp / 100;

    if (c_data.target_temp < -MAX_NEGATIVE_TEMP)
    {
        c_data.set_speed = 0;
        return;
    }

    /* this pid func assumes current_temp and target_temp are in deg celcius */
    if (cur_temp > tgt_temp + MAX_DELTA_DEG_C)
    {
        c_data.set_speed = MAX_RPM;
    }
    else if (cur_temp >= tgt_temp)
    {
        c_data.set_speed = (MAX_RPM / 100) - (((MAX_RPM - MIN_RPM) / (MAX_DELTA_DEG_C * 100))
                * (MAX_DELTA_DEG_C + tgt_temp - cur_temp));
        c_data.set_speed *= 100;
    }
    else if (cur_temp < tgt_temp)
    {
        c_data.set_speed = 0;
    }

    if (c_data.set_speed < MIN_RPM)
    {
        c_data.set_speed = 0;
    }
    
    if (c_data.set_speed > MAX_RPM)
    {
        c_data.set_speed = MAX_RPM;
    }
}

#define EFFECTIVE_INDEX(i, max)   (i % max)
/* RX thread */
static void com_rx(void)
{
    while (true) {
        /* wait for incoming data */
        k_sem_take(&rx_sem, K_FOREVER);

        /* current temperature update constantly */
        get_current_temp_internal(&c_data.current_temp);

        if (c_rx_cnt >= 16)
        {
            while ((c_rx_buf[c_rx_rd_idx] != 0xAA) && (c_rx_cnt >= 16))
            {
                info_log("no");
                rx_buf_get(1);
            }

            if (c_rx_cnt < 16)
            {
                continue;
            }

            info_log("y, 0x%x", c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 15, COMP_RX_BUFFER_SIZE)]);
            if (c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 15, COMP_RX_BUFFER_SIZE)] == 0x55)
            {
                info_log("yes");
                c_data.c_rx_data.comp_cur_speed = 
                    ((c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 3, COMP_RX_BUFFER_SIZE)] << 8)
                        | c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 2, COMP_RX_BUFFER_SIZE)]);
                c_data.c_rx_data.comp_cur_current = 
                    ((c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 5, COMP_RX_BUFFER_SIZE)] << 8)
                    | c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 4, COMP_RX_BUFFER_SIZE)]);
                c_data.c_rx_data.bus_volt = 
                    ((c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 7, COMP_RX_BUFFER_SIZE)] << 8)
                        | c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 6, COMP_RX_BUFFER_SIZE)]);
                c_data.c_rx_data.drv_temp = c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 8, COMP_RX_BUFFER_SIZE)];
                c_data.c_rx_data.drv_err = c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 9, COMP_RX_BUFFER_SIZE)];
                c_data.c_rx_data.drv_cnd = c_rx_buf[EFFECTIVE_INDEX(c_rx_rd_idx + 10, COMP_RX_BUFFER_SIZE)];

                rx_buf_get(16);

                /* check rpm needed */
                pid_run();

                info_log("ss: %d, cs: %d, cc: %d, bv: %d, dt: %d, de: %d, dc: %d",
                        c_data.set_speed, c_data.c_rx_data.comp_cur_speed,
                        c_data.c_rx_data.comp_cur_current, c_data.c_rx_data.bus_volt,
                        c_data.c_rx_data.drv_temp, c_data.c_rx_data.drv_err,
                        c_data.c_rx_data.drv_cnd);

                /* send host frame */
                if (c_data.set_speed <= MIN_RPM)
                {
                    bldc_stop();
                }
                else if (c_data.set_speed <= MAX_RPM)
                {
                    bldc_start(c_data.set_speed);
                }
                else
                {
                    error_log("something wrong with pid loop, speed out of range");
                }
            }
            else
            {
                info_log("no2");
                rx_buf_get(1);
            }
        }

        /* give up time if we have a solid stream of data */
        k_yield();
    }
}

void compressor_set_target_temp(int32_t temp)
{
    c_data.target_temp = temp;
}

int compressor_init(struct device* compressor_dev, struct device* gpio)
{
    int ret = 0;

    if (compressor_dev == NULL)
    {
        error_log("invalid compressor device");
        return -1;
    }

    /* enable 5V conversion */
    gpio_pin_configure(gpio, UART_5V_ENABLE_PIN, GPIO_DIR_OUT);
    gpio_pin_write(gpio, UART_5V_ENABLE_PIN, 1);

    k_sem_init(&rx_sem, 0, 1);

    uart_irq_rx_disable(compressor_dev);
    uart_irq_tx_disable(compressor_dev);
    com_uart_flush(compressor_dev);
    uart_irq_callback_set(compressor_dev, com_uart_isr);
    uart_irq_rx_enable(compressor_dev);

    memset(&c_data, 0, sizeof(struct compressor_data));

    c_data.compressor_dev = compressor_dev;
    c_data.gpio_dev = gpio;

    /* get current_temp_ptr */
    get_current_temp_internal(&c_data.current_temp);

    /* start RX thread */
    k_thread_create(&com_rx_thread, com_rx_stack,
            K_THREAD_STACK_SIZEOF(com_rx_stack),
            (k_thread_entry_t)com_rx, NULL, NULL, NULL,
            K_PRIO_COOP(COMPRESSOR_UART_INT_PRIO), 0, K_NO_WAIT);

    bldc_start(MAX_RPM);

    info_log("compressor started\n");

    return 0;
}

void get_compressor_data(struct compressor_rx_data** comp)
{
    *comp = &c_data.c_rx_data;
}

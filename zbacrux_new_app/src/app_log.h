#ifndef __APP_LOG_H__
#define __APP_LOG_H__

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <sys/printk.h>

/* FIXME move to appropriate place */
#define LOG_STDOUT

#ifdef LOG_STDOUT
#define error_log(fmt, ...)   printk(fmt"\n", ##__VA_ARGS__)
#define info_log(fmt, ...)    printk(fmt"\n", ##__VA_ARGS__)
#endif

#endif

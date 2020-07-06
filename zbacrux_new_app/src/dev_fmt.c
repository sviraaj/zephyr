#include "dev_fmt.h"
#include <string.h>

#define MAX_PUSH_DATA_SZ     512
#define MAX_CONV_BUF_SZ      64

static u8_t __aligned(4) push_data_buf[MAX_PUSH_DATA_SZ];
static u8_t __aligned(4) fetch_data_buf[MAX_PUSH_DATA_SZ];
static u8_t __aligned(4) convert_buf[MAX_CONV_BUF_SZ];

int get_dev_data_batch(u8_t* data_json, u16_t len)
{
    u16_t got_len = 0;
    int ret = 0;
    int s_ret = 0;

    /* FIXME Needed? */
    memset(data_json, 0, len);

    fetch_unit_start();

    /* FIXME why extra MAX_PUSH_DATA_SZ left ? Hack */
    while (got_len < (len - MAX_PUSH_DATA_SZ))
    {
        memset(fetch_data_buf, 0, MAX_PUSH_DATA_SZ);

        ret = fetch_data_unit(fetch_data_buf, MAX_PUSH_DATA_SZ);
        if (ret != 0)
        {
            if (ret == -ENOSPC)
            {
                /* exhausted all elements */
                ret = 0;
            }
            break;
        }
 
#if 0
        info_log("Fetch: %d", strlen(fetch_data_buf));
        info_log("%s", fetch_data_buf);
        info_log("%s\n", fetch_data_buf + 128);
        k_sleep(100);
#endif

        s_ret = snprintf(data_json + got_len, (len - got_len), "%s,", fetch_data_buf);
        if (s_ret < 0 || s_ret >= (len - got_len))
        {
            break;
        }

        if(memcmp(data_json + got_len, fetch_data_buf, strlen(fetch_data_buf)) == 0)
        {
            info_log("allll gooooodddd!!!");
        }
        got_len = strlen(data_json);
    }

    /* Remove ending "," for last element */
    data_json[got_len - 1] = 0;

#if 0
    info_log("strlen data json %d %d", strlen(data_json), got_len);
    for (int i = 0; i < strlen(data_json); i+=128)
    {
        info_log("%s", data_json + (i * 128));
        k_sleep(100);
    }
#endif

    return ret;
}

static u8_t* convert_to_str(u8_t idx, void* data)
{
    struct env_data* env;
    struct imu_data* imu;
    struct mech_data* mch;
    struct elec_data* elc;

    switch(idx)
    {
        case T1:
            env = (struct env_data*) data;
            snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d",
                    env->ambient_temp);
            break;
        case T2:
            env = (struct env_data*) data;
            snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d",
                    env->internal_temp);
            break;
        case HP1:
            env = (struct env_data*) data;
            snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d",
                    env->ambient_hum, env->ambient_pres);
            break;
        case HP2:
            env = (struct env_data*) data;
            snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d",
                    env->internal_hum, env->internal_pres);
            break;
        case IM1:
            imu = (struct imu_data*) data;
            snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    imu->roll, imu->pitch, imu->yaw,
                    imu->accel_x, imu->accel_y, imu->accel_z,
                    imu->gyro_x, imu->gyro_y, imu->gyro_z);
            break;
        case L1:
            break;
        case GP:
            break;
        case M:
            mch = (struct mech_data*) data;
            snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d,%d,%d",
                    mch->rpm,
                    mch->temp,
                    mch->door_open_time,
                    mch->err_code);
            break;
        case B:
            elc = (struct elec_data*) data;
            snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d,%d,%d,%d,%d,%d,%d",
                    elc->voltage_mv,
                    elc->current_mch,
                    elc->current_brd,
                    elc->led_on_time,
                    elc->fan_on_time,
                    elc->compressor_on_time,
                    elc->display_on_time,
                    elc->io_cnt);
            break;
        default:
            error_log("unknown convert request");
            break;
    }

    return convert_buf;
}

int convert_data_to_json(struct device_data* data)
{
    //k_sem_take(&g_dev_data.dev_data.sem_data_sample);

    //get_dev_data(struct device_data** dev_data);

    u16_t add_len = 0;

    memset(push_data_buf, 0, MAX_PUSH_DATA_SZ);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len, OPEN_DEV_FMT);
    add_len = strlen(push_data_buf);

    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len, SINGLE_DT_ENTRY("DT")",",
            data->sys_time);
    add_len = strlen(push_data_buf);

    snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d",
            data->env.ambient_temp);
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len, SINGLE_DEV_ENTRY("T1")",",
            convert_buf);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d",
            data->env.internal_temp);
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("T2")",",
            convert_buf);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d",
            data->env.ambient_hum, data->env.ambient_pres);
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("HP1")",",
            convert_buf);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d",
            data->env.internal_hum, data->env.internal_pres);
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("HP2")",",
            convert_buf);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
            data->imu.roll, data->imu.pitch, data->imu.yaw,
            data->imu.accel_x, data->imu.accel_y, data->imu.accel_z,
            data->imu.gyro_x, data->imu.gyro_y, data->imu.gyro_z);
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("IM1")",",
            convert_buf);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

#if 0
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("L1")",",
            convert_to_str(T1, data->env));
    add_len = strlen(push_data_buf);
#endif

    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("GP")",",
            data->gps.batch_gps);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d,%d,%d",
            data->mch.rpm, data->mch.temp, data->mch.door_open_time, data->mch.err_code);
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("M")",",
            convert_buf);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(convert_buf, MAX_CONV_BUF_SZ, "%d,%d,%d,%d,%d,%d,%d,%d",
            data->elc.voltage_mv, data->elc.current_mch, data->elc.current_brd,
            data->elc.led_on_time, data->elc.fan_on_time, data->elc.compressor_on_time,
            data->elc.display_on_time, data->elc.io_cnt);
    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("B")",",
            convert_buf);
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len,  SINGLE_DEV_ENTRY("TG"),
            "");
    add_len = strlen(push_data_buf);
    memset(convert_buf, 0, MAX_CONV_BUF_SZ);

    snprintf(push_data_buf + add_len, MAX_PUSH_DATA_SZ - add_len, CLOSE_DEV_FMT);
    add_len = strlen(push_data_buf);

    info_log("Pushing:");
    info_log("%s", push_data_buf);
    info_log("%s", push_data_buf + 128);
    info_log("Pushing done");

    /* push the json dat to memory */
    return push_data_unit(push_data_buf, MAX_PUSH_DATA_SZ);
}

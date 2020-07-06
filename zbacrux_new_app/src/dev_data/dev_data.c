#include "dev_data.h"
#include "app_compressor.h"
#include "config_params.h"
#include "app_cfg.h"
#include "dev_fmt.h"
#include <fs/fcb.h>
#include <drivers/sensor.h>
#include <drivers/modem/a9grda.h>
#include <string.h>
#include <math.h>

#define WORD_ALIGNED(x)  ((u8_t) ((((u32_t)x) & 0x3) == 0))

static struct fcb data_fcb;
static struct fcb_entry fetch_loc = {0};
static struct fcb_entry next_fetch_loc = {0};
static struct fcb_entry push_loc;

static struct m_dev_data g_dev_data;
static struct device_params* dev_params = NULL;
static struct global_params* g_params = NULL;
//static u8_t emergency_flag = false;


static void env_fetch(struct k_timer *dummy);
static void gps_fetch(struct k_timer *dummy);
static void elec_fetch(struct k_timer *dummy);
static void imu_fetch(struct k_timer *dummy);
static void mech_fetch(struct k_timer *dummy);
static void misc_fetch(struct k_timer *dummy);
static void data_send_cb(struct k_timer *dummy);

K_TIMER_DEFINE(env_timer, env_fetch, NULL);
K_TIMER_DEFINE(gps_timer, gps_fetch, NULL);
K_TIMER_DEFINE(elec_timer, elec_fetch, NULL);
K_TIMER_DEFINE(imu_timer, imu_fetch, NULL);
K_TIMER_DEFINE(mech_timer, mech_fetch, NULL);
K_TIMER_DEFINE(misc_timer, misc_fetch, NULL);
K_TIMER_DEFINE(data_send_timer, data_send_cb, NULL);

static struct flash_sector fcb_sector_list[] = {
	[0] = {
		.fs_off = 0x8000,
		.fs_size = 0xC0000, /* 768 K */
	},
	[1] = {
		.fs_off = 0xC8000,
		.fs_size = 0xC0000
	},
	[2] = {
		.fs_off = 0x188000,
		.fs_size = 0xC0000
	},
	[3] = {
		.fs_off = 0x248000,
		.fs_size = 0xC0000
	}
};

static void imu_fetch_update(struct device_data* dev_d)
{
    int ret = 0;
    struct sensor_value accel[3], gyro[3];
    //u8_t threshold_check = 0;

    k_sem_take(&g_dev_data.dev_data.i2c_lock, K_FOREVER);

    /* IMU data fetch */
    if (g_dev_data.devs.imu_dev != NULL)
    {
        ret = sensor_sample_fetch(g_dev_data.devs.imu_dev);
        if (ret != 0)
        {
            error_log("sample fetch imu fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        if (ret != 0)
        {
            error_log("chan accel fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        if (ret != 0)
        {
            error_log("chan gyro fail %d", ret);
            goto ret;
        }

#if 0
        info_log("accel x: %d.%06d, y: %d.%06d, z: %d.%06d;"
               "gyro x: %d.%06d; y: %d.%06d, z: %d.%06d\n",
               accel[0].val1, accel[0].val2, accel[1].val1,
               accel[1].val2, accel[2].val1, accel[2].val2,
               gyro[0].val1, gyro[0].val2, gyro[1].val1, gyro[1].val2,
               gyro[2].val1, gyro[2].val2);
#endif
        dev_d->imu.accel_x = ((accel[0].val1 * 1000) + (int32_t)(accel[0].val2 / 1000));
        dev_d->imu.accel_y = ((accel[1].val1 * 1000) + (int32_t)(accel[1].val2 / 1000));
        dev_d->imu.accel_z = ((accel[2].val1 * 1000) + (int32_t)(accel[2].val2 / 1000));
                          
        dev_d->imu.gyro_x = ((gyro[0].val1 * 1000) + (int32_t)(gyro[0].val2 / 1000));
        dev_d->imu.gyro_y = ((gyro[1].val1 * 1000) + (int32_t)(gyro[1].val2 / 1000));
        dev_d->imu.gyro_z = ((gyro[2].val1 * 1000) + (int32_t)(gyro[2].val2 / 1000));
                          
        dev_d->imu.roll  = (atan(dev_d->imu.accel_x / sqrt(pow(dev_d->imu.accel_y, 2) +
                        pow(dev_d->imu.accel_z, 2))) * 18000);
        dev_d->imu.pitch = (atan(dev_d->imu.accel_y / sqrt(pow(dev_d->imu.accel_x, 2) +
                        pow(dev_d->imu.accel_z, 2))) * 18000);
        dev_d->imu.yaw = 0; /* FIXME */

        info_log("accel x: %d, y: %d, z: %d;"
               "gyro x: %d; y: %d, z: %d\n",
               dev_d->imu.accel_x, dev_d->imu.accel_y, dev_d->imu.accel_z,
               dev_d->imu.gyro_x, dev_d->imu.gyro_y, dev_d->imu.gyro_z);
    }

#if 0
    /* check for variations above threshold */
    threshold_check = imu_check_priority(dev_d->imu);
    if (threshold_check != 0)
    {}
#endif

ret:
    k_sem_give(&g_dev_data.dev_data.i2c_lock);
}

static void env_fetch_update(struct device_data* dev_d)
{
    int ret = 0;
    struct sensor_value temp, press, humidity;

    k_sem_take(&g_dev_data.dev_data.i2c_lock, K_FOREVER);

    if (g_dev_data.devs.env1_dev != NULL)
    {
        /* Temperature data fetch */
        ret = sensor_sample_fetch(g_dev_data.devs.env1_dev);
        if (ret != 0)
        {
            error_log("sample fetch env fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.env1_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        if (ret != 0)
        {
            error_log("chan temp env fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.env1_dev, SENSOR_CHAN_PRESS, &press);
        if (ret != 0)
        {
            error_log("chan press env fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.env1_dev, SENSOR_CHAN_HUMIDITY, &humidity);
        if (ret != 0)
        {
            error_log("chan hum env fail %d", ret);
            goto ret;
        }

        dev_d->env.ambient_temp = (int32_t) (((temp.val1 * 100) + (int)(temp.val2 / 10000)));
        dev_d->env.ambient_hum = (int32_t) (((humidity.val1 * 100) + (int)(humidity.val2 / 10000)));
        dev_d->env.ambient_pres = (int32_t) (((press.val1 * 100) + (int)(press.val2 / 10000)));

#if 0
        info_log("amb temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
              temp.val1, temp.val2, press.val1, press.val2,
              humidity.val1, humidity.val2);
#endif
        info_log("ambient tmp: %d, hum: %d, pres: %d", dev_d->env.ambient_temp,
                dev_d->env.ambient_hum, dev_d->env.ambient_pres);
    }

    if (g_dev_data.devs.env2_dev != NULL)
    {
        ret = sensor_sample_fetch(g_dev_data.devs.env2_dev);
        if (ret != 0)
        {
            error_log("sample fetch env fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.env2_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        if (ret != 0)
        {
            error_log("chan temp env fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.env2_dev, SENSOR_CHAN_PRESS, &press);
        if (ret != 0)
        {
            error_log("chan press env fail %d", ret);
            goto ret;
        }
        ret = sensor_channel_get(g_dev_data.devs.env2_dev, SENSOR_CHAN_HUMIDITY, &humidity);
        if (ret != 0)
        {
            error_log("chan hum env fail %d", ret);
            goto ret;
        }

        dev_d->env.internal_temp = (int32_t) (((temp.val1 * 100) + (int)(temp.val2 / 10000)));
        dev_d->env.internal_hum = (int32_t) (((humidity.val1 * 100) + (int)(humidity.val2 / 10000)));
        dev_d->env.internal_pres = (int32_t) (((press.val1 * 100) + (int)(press.val2 / 10000)));

#if 0
        info_log("int temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
              temp.val1, temp.val2, press.val1, press.val2,
              humidity.val1, humidity.val2);
#endif
        info_log("internal tmp: %d, hum: %d, pres: %d", dev_d->env.internal_temp,
                dev_d->env.internal_hum, dev_d->env.internal_pres);
    }

ret:
    k_sem_give(&g_dev_data.dev_data.i2c_lock);
}

static void gps_fetch_update(struct device_data* dev_d)
{
    int ret = 0;
    struct usr_gps_cfg gps_cfg;

    if (g_dev_data.devs.gps_dev == NULL)
    {
        return;
    }

    gps_cfg.gps_data = dev_d->gps.batch_gps;

    info_log("GPS_read");

    /* GPS data fetch */
    ret = mdm_a9g_gps_read(g_dev_data.devs.gps_dev, &gps_cfg);
    if (ret != 0)
    {
        error_log("fail a9g gps read %d", ret);
        return;
    }

    memcpy(dev_d->gps.batch_gps, gps_cfg.gps_data, MAX_GPS_BATCH_SIZE);

    info_log("GPS: %s\n", dev_d->gps.batch_gps);
}


static void mech_fetch_update(struct device_data* dev_d)
{
    struct compressor_rx_data* c_data = NULL;

    get_compressor_data(&c_data);

    dev_d->mch.rpm = c_data->comp_cur_speed;
    dev_d->mch.temp = c_data->drv_temp;
    dev_d->mch.err_code = c_data->drv_err;
}

static void elec_fetch_update(struct device_data* dev_d)
{
    struct compressor_rx_data* c_data = NULL;

    get_compressor_data(&c_data);

    dev_d->elc.current_mch = c_data->comp_cur_current;
    dev_d->elc.voltage_mv = c_data->bus_volt;
}

static void misc_fetch_update(struct device_data* dev_d)
{

}

static void convert_to_json_worker(struct k_work *work)
{
    struct m_dev_data *data =
                CONTAINER_OF(work, struct m_dev_data, convert_to_json_work);
    struct device_data* dev_d = &data->dev_data;

    convert_data_to_json(dev_d);
}

static void env_worker(struct k_work *work)
{
    /* do the processing that needs to be done periodically */
    struct m_dev_data *data =
                CONTAINER_OF(work, struct m_dev_data, env_imu_work);
    struct device_data* dev_d = &data->dev_data;

    led_toggle();

    imu_fetch_update(dev_d);

    /* Check for conditional value callbacks */
    
}

static void gps_worker(struct k_work *work)
{
    struct m_dev_data *data =
                CONTAINER_OF(work, struct m_dev_data, gps_mech_work);
    struct device_data* dev_d = &data->dev_data;

    gps_fetch_update(dev_d);

    /* Check for conditional value callbacks */
}

static void elec_worker(struct k_work *work)
{
    struct m_dev_data *data =
                CONTAINER_OF(work, struct m_dev_data, elec_misc_work);
    struct device_data* dev_d = &data->dev_data;

    elec_fetch_update(dev_d);

    /* Check for conditional value callbacks */
}

static void imu_worker(struct k_work *work)
{
    /* do the processing that needs to be done periodically */
    struct m_dev_data *data =
                CONTAINER_OF(work, struct m_dev_data, env_imu_work);
    struct device_data* dev_d = &data->dev_data;

    led_toggle();

    imu_fetch_update(dev_d);

    /* Check for conditional value callbacks */
}

static void mech_worker(struct k_work *work)
{
    struct m_dev_data *data =
                CONTAINER_OF(work, struct m_dev_data, gps_mech_work);
    struct device_data* dev_d = &data->dev_data;

    mech_fetch_update(dev_d);

    /* Check for conditional value callbacks */
}

static void misc_worker(struct k_work *work)
{
    struct m_dev_data *data =
                CONTAINER_OF(work, struct m_dev_data, elec_misc_work);
    struct device_data* dev_d = &data->dev_data;

    misc_fetch_update(dev_d);

    /* Check for conditional value callbacks */
}

static void env_fetch(struct k_timer *dummy)
{
    k_work_submit(&g_dev_data.env_work);
}

static void gps_fetch(struct k_timer *dummy)
{
    k_work_submit(&g_dev_data.gps_work);
}

static void elec_fetch(struct k_timer *dummy)
{
    k_work_submit(&g_dev_data.elec_work);
}

static void imu_fetch(struct k_timer *dummy)
{
    k_work_submit(&g_dev_data.imu_work);
}

static void mech_fetch(struct k_timer *dummy)
{
    k_work_submit(&g_dev_data.mech_work);
}

static void misc_fetch(struct k_timer *dummy)
{
    k_work_submit(&g_dev_data.misc_work);
}

static void data_send_instance(struct device_data* dev_data)
{
    /* FIXME Hack */
#if 0
    imu_fetch_update(dev_data);

    env_fetch_update(dev_data);

    gps_fetch_update(dev_data);

    mech_fetch_update(dev_data);

    elec_fetch_update(dev_data);

    misc_fetch_update(dev_data);
#endif

    dev_data->sys_time = k_cycle_get_32();

    /* FIXME Hack */
    memset(g_params->sys_datetime, 0, MAX_VERSION_SIZE);
    snprintf(g_params->sys_datetime, MAX_VERSION_SIZE,
            "%d", dev_data->sys_time);

    /* sample done, post sem */
    //k_sem_give(&g_dev_data.dev_data.sem_data_sample);
}

static void data_send_cb(struct k_timer *dummy)
{
    info_log("send data instance");

    data_send_instance(&g_dev_data.dev_data);

    k_work_submit(&g_dev_data.convert_to_json_work);
}

int dev_data_init(struct sensor_dev* devs)
{
    struct usr_gps_cfg gps_cfg;
    int ret = 0;

    /* get device and global param pointers */
    get_device_params(&dev_params);
    if (dev_params == NULL)
    {
        error_log("invalid params dev");
    }

    get_global_params(&g_params);
    if (g_params == NULL)
    {
        error_log("invalid params global");
    }

    data_fcb.f_sectors = fcb_sector_list;
    data_fcb.f_sector_cnt = 4;
    data_fcb.f_magic = 0;

    ret = fcb_init(DT_FLASH_AREA_EXT_STORAGE_ID, &data_fcb);
    if (ret != 0)
    {
        error_log("failed to initialise fcb storage, %d", ret);
        return ret;
    }

    /* FIXME Temporary */
    ret = fcb_clear(&data_fcb);
    if (ret != 0)
    {
        error_log("fcb_clear failed");
    }

    /* Fetch the oldest element */
    fetch_loc.fe_elem_off = 0;

    /* Initialise sem */
    k_sem_init(&g_dev_data.dev_data.sem_data_sample, 0, 1);
    k_sem_init(&g_dev_data.dev_data.i2c_lock, 1, 1);

    /* FIXME start periodic timer that expires once every int */
    k_work_init(&g_dev_data.env_work, env_worker);
    k_work_init(&g_dev_data.gps_work, gps_worker);
    k_work_init(&g_dev_data.elec_work, elec_worker);
    k_work_init(&g_dev_data.imu_work, imu_worker);
    k_work_init(&g_dev_data.mech_work, mech_worker);
    k_work_init(&g_dev_data.misc_work, misc_worker);
    k_work_init(&g_dev_data.convert_to_json_work, convert_to_json_worker);

    g_dev_data.devs.env1_dev = devs->env1_dev;
    g_dev_data.devs.env2_dev = devs->env2_dev;
    g_dev_data.devs.imu_dev = devs->imu_dev;
    g_dev_data.devs.gps_dev = devs->gps_dev;

    if (g_dev_data.devs.gps_dev != NULL)
    {
        /* GPS */
        mdm_a9g_gps_init(g_dev_data.devs.gps_dev, &gps_cfg);
        //mdm_a9g_gps_agps(g_dev_data.devs.gps_dev, &gps_cfg);
    }
    else
    {
        error_log("no gps dev found");
    }

    if (devs->imu_dev == NULL)
    {
        error_log("no imu dev found");
    }

    if (devs->env1_dev == NULL)
    {
        error_log("no env1 dev found");
    }

    if (devs->env2_dev == NULL)
    {
        error_log("no env2 dev found");
    }

    /* IMU */

    /* Analog sensor */

    k_timer_start(&env_timer, K_SECONDS(ENV_INT), K_SECONDS(ENV_INT));
    k_timer_start(&gps_timer, K_SECONDS(GPS_INT), K_SECONDS(GPS_INT));
    k_timer_start(&elec_timer, K_SECONDS(ELEC_INT), K_SECONDS(ELEC_INT));
    k_timer_start(&imu_timer, K_SECONDS(IMU_INT), K_SECONDS(IMU_INT));
    k_timer_start(&mech_timer, K_SECONDS(MECH_INT), K_SECONDS(MECH_INT));
    k_timer_start(&misc_timer, K_SECONDS(MISC_INT), K_SECONDS(MISC_INT));
    k_timer_start(&data_send_timer, K_MSEC(dev_params->data_interval),
            K_MSEC(dev_params->data_interval));

    info_log("dev data initialisation done");

    return 0;
}

/* call this once to get pointer */
int get_dev_data(struct device_data** dev_data)
{
    *dev_data = &g_dev_data.dev_data;

    return 0;
}

int push_data_unit(u8_t *buf, u16_t len)
{
    int ret = 0;

    /* buf and len should be word aligned */
    if (WORD_ALIGNED(buf) != 1 || WORD_ALIGNED(len) != 1)
    {
        error_log("failed to push unaligned data");
    }

    ret = fcb_free_sector_cnt(&data_fcb);
    if (ret == 0)
    {
        error_log("push and fetch indexes reached same sector");
        //return -1;
    }

    ret = fcb_append(&data_fcb, len, &push_loc);
    if (ret == -ENOSPC)
    {
        info_log("fcb data overwrite");
        ret = fcb_rotate(&data_fcb);
        if (ret != 0)
        {
            error_log("failed to rotate fcb data");
            return ret;
        }
        ret = fcb_append(&data_fcb, len, &push_loc);
    }

    if (ret != 0)
    {
        error_log("failed to push data to fcb storage");
        return ret;
    }

    ret = flash_area_write(data_fcb.fap, FCB_ENTRY_FA_DATA_OFF(push_loc),
            buf, len);
    if (ret != 0)
    {
        error_log("failed to write data");
        return ret;
    }

    ret = fcb_append_finish(&data_fcb, &push_loc);
    if (ret != 0)
    {
        error_log("append finish fail to fcb storage");
        return ret;
    }

    return ret;
}

int fetch_data_unit(u8_t* buf, u16_t len)
{
    int ret = 0;

    /* buf and len should be word aligned */
    if (WORD_ALIGNED(buf) != 1 || WORD_ALIGNED(len) != 1)
    {
        error_log("failed to push unaligned data");
    }

    ret = fcb_is_empty(&data_fcb);
    if (ret != 0)
    {
        info_log("empty fcb");
        /* FIXME should be no element */
        return -ENOSPC;
    }

    /* reached end of all elements */
    if ((push_loc.fe_sector == next_fetch_loc.fe_sector) &&
            (push_loc.fe_elem_off == next_fetch_loc.fe_elem_off))
    {
        return -ENOSPC;
    }

    ret = fcb_getnext(&data_fcb, &next_fetch_loc);
    if (ret != 0)
    {
        error_log("failed to get dev data by fcb");

#if 0
        if (fcb_offset_last_n(&data_fcb, 1, &next_fetch_loc) != 0)
        {
            error_log("fcb unable to fetch last entry");
        }
#endif
        return ret;
    }

	ret = flash_area_read(data_fcb.fap,
			     FCB_ENTRY_FA_DATA_OFF(next_fetch_loc),
			     buf, len);
    if (ret != 0)
    {
        error_log("failed to read next dev data");
    }

    return ret;
}

void fetch_unit_start()
{
    fetch_loc = next_fetch_loc;
}

void fetch_unit_rollback()
{
    next_fetch_loc = fetch_loc;
}

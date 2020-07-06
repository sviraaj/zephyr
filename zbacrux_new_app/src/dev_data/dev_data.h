#ifndef __DEV_DATA_H__
#define __DEV_DATA_H__

#include "app_cfg.h"

#define MAX_COORDINATE_SIZE 16U
#define MAX_GPS_BATCH_SIZE 128U

/* Interval in seconds */
#define ENV_INT 1
#define GPS_INT 30
#define ELEC_INT 60
#define IMU_INT 1
#define MECH_INT 30
#define MISC_INT 60

enum data_sources {
    ENV_DATA = 0,
    IMU_DATA,
    GPS_DATA,
    MECH_DATA,
    ELEC_DATA,
    MISC_DATA,
};

struct env_data {
    /* (temperature * 100) deg C */
    int32_t ambient_temp;
    int32_t internal_temp;
    /* (humidity %)  */
    int32_t ambient_hum;
    int32_t internal_hum;
    /* (presssure * 100) */
    int32_t ambient_pres;
    int32_t internal_pres;
    /* reserved */
    //u16_t reserved[8];
};

struct gps_data {
#if 0
    char lat[MAX_COORDINATE_SIZE];
    char lon[MAX_COORDINATE_SIZE];
    char speed[MAX_COORDINATE_SIZE];
    u16_t sats_tracked;
    u16_t fix_type;
#endif
    /* Send the NMEA as is */
    char batch_gps[MAX_GPS_BATCH_SIZE];
};

struct imu_data {
    /* x1000 */
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
};

struct mech_data {
    u16_t rpm;
    u16_t temp;
    u16_t door_open_time;
    u8_t err_code;
    u8_t res;
    u16_t reserved[8];
};

struct elec_data {
    /* battery voltage in mv */
    u16_t voltage_mv;
    /* battery precent as per voltage */
    u16_t percent;
    /* current consumption of mechanical components */
    u16_t current_mch;
    /* current consumption of electrical components */
    u16_t current_brd;
    /* led of the internal chamber on time in seconds */
    u16_t led_on_time;
    /* fan on time in seconds; mechanical */
    u16_t fan_on_time;
    /* compressor rpm related; check validity */
    u16_t compressor_on_time;
    /* measure of display activity */
    u16_t display_on_time;
    /* measure of user io activity with the device */
    u16_t io_cnt;
};

struct device_data {
    u32_t sys_time;
    struct gps_data gps;
    struct env_data env;
    struct imu_data imu;
    struct mech_data mch;
    struct elec_data elc;

    /* sem for indicating when a sample is done */
    struct k_sem sem_data_sample;

    struct k_sem i2c_lock;
};

struct sensor_dev
{
    struct device *env1_dev;
    struct device *env2_dev;
    struct device *imu_dev;
    struct device *gps_dev;
};

struct m_dev_data
{
    struct device_data dev_data;

    struct sensor_dev devs;

    /* FIXME add generic support instead of clubbing like this. Hack for now */
    struct k_work env_work;
    struct k_work gps_work;
    struct k_work elec_work;
    struct k_work imu_work;
    struct k_work mech_work;
    struct k_work misc_work;

    struct k_work convert_to_json_work;
};

/* dev data init function to initialse device data */
int dev_data_init(struct sensor_dev* devs);
/* 
 * call this function after sem_dev_sample is updated
 * to get latest data in packed struct form
 */
int get_dev_data(struct device_data** dev_data);
/*
 * this function is used to push the data to storage fcb
 * in json format, which is formed by another function
 * which fetches the struct data form and forms a json
 * obj from the data and then calls this function to store
 */
int push_data_unit(u8_t *buf, u16_t len);
/* 
 * this function is used to fetch single data point stored
 * in fcb in json form
 */
int fetch_data_unit(u8_t* buf, u16_t len);

/*
 * fetch unit start indication
 */
void fetch_unit_start();

/*
 * fetch unit can be updated to fetch from next entry
 * as the work with that data is done
 */
//void fetch_unit_done();

/*
 * fetch unit can be rollbacked to fetch again from the same entry
 * as the work with that data has failed
 */
void fetch_unit_rollback();

static inline void get_current_temp_internal(int32_t* temp)
{
    struct device_data* dev_data;

    get_dev_data(&dev_data);

    *temp = dev_data->env.internal_temp;
}

static inline void get_current_temp_ambient(int32_t* temp)
{
    struct device_data* dev_data;

    get_dev_data(&dev_data);

    *temp = dev_data->env.ambient_temp;
}


struct data_evt
{
    /* id of the evt */
    u32_t id;
    /* type of evt */
    u16_t type;
    /* status of the cb */
    u16_t status;
};

typedef void (*data_evt_cb)(struct data_evt* evt, void* data);

struct data_src
{
    int (*init)(void);
    int (*get_data)(void* data);
    int (*get_data_str)(u8_t* data_buf, u16_t len);
    int (*register_cb)(data_evt_cb cb, void* cb_ctx);
    int (*remove_cb)(data_evt_cb cb);
};



#endif

#ifndef __SENSOR_H__
#define __SENSOR_H__

#define MAX_COORDINATE_SIZE 16U

struct env_data {
	u16_t ambient_tmp; /* (temperature * 100) deg C */
	u16_t internal_tmp; /* (temperature * 100) deg C */
	u16_t ambient_hum; /* (humidity * 100) deg C */
	u16_t internal_hum; /* (humidity * 100) deg C */
	u16_t reserved[8]; /* reserved */
};

struct gps_data {
	char lat[MAX_COORDINATE_SIZE];
	char lon[MAX_COORDINATE_SIZE];
	char speed[MAX_COORDINATE_SIZE];
	u16_t sats_tracked;
	u16_t fix_type;
};

struct imu_data {
	/* x1000*/
	u16_t roll;
	u16_t pitch;
	u16_t yaw;
	u16_t accel_x;
	u16_t accel_y;
	u16_t accel_z;
	u16_t gyro_x;
	u16_t gyro_y;
	u16_t gyro_z;
};

struct mech_data {
	u16_t rpm;
	u16_t temp;
	u16_t door_open_time;
	u16_t reserved[8];
};

struct elec_data {
	u16_t voltage;
	u16_t current_mch;
	u16_t current_brd;
	u16_t led_on_time;
	u16_t fan_on_time;
	u16_t compressor_on_time;
	u16_t display_on_time;
	u16_t io_cnt;
};

struct bat_data
{
    u16_t percent;
    u16_t volt_mv;
};

struct sensor_data {
	struct gps_data gps;
	struct env_data env;
	struct imu_data imu;
	struct mech_data mch;
	struct elec_data elc;
    struct bat_data bat;
};

extern struct device *imu_dev;

#endif

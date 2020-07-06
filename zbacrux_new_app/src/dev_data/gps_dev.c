#include <drivers/modem/a9grda.h>
#include "dev_data.h"
#include "config_params.h"
#include "app_cfg.h"
#include "dev_fmt.h"
#include <string.h>
#include <math.h>

#define GPS_AGPS_ENABLE    0

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

static struct device* gps_dev;
struct gps_data gps; 

int gps_init(void)
{
    int ret = 0;
    struct usr_gps_cfg gps_cfg;

    if (gps_dev != NULL)
    {
        ret = mdm_a9g_gps_init(gps_dev, &gps_cfg);
        if (ret != 0)
        {
            goto exit;
        }
#if GPS_AGPS_ENABLE
        ret = mdm_a9g_gps_agps(gps_dev, &gps_cfg);
        if (ret != 0)
        {
            goto exit;
        }
#endif
    }

exit:
    return ret;
}

int gps_fetch_data(void* data)
{
    struct gps_data* gps = (struct gps_data*) data;
    int ret = 0;
    struct usr_gps_cfg gps_cfg;

    if (gps == NULL)
    {
        return -EINVAL;
    }

    if (gps_dev == NULL)
    {
        error_log("GPS dev NULL");
        return -EINVAL;
    }

    gps_cfg.gps_data = dev_d->gps.batch_gps;

    /* GPS data fetch */
    ret = mdm_a9g_gps_read(g_dev_data.devs.gps_dev, &gps_cfg);
    if (ret != 0)
    {
        error_log("fail a9g gps read %d", ret);
        return -EIO;
    }

    memcpy(gps->batch_gps, gps_cfg.gps_data, MAX_GPS_BATCH_SIZE);

    info_log("GPS: %s\n", dev_d->gps.batch_gps);

    return 0;
}

int get_gps_str(u8_t* data_buf, u16_t len)
{

}

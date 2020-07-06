#ifndef __MDM_QUECTEL_BG95_H__
#define __MDM_QUECTEL_BG95_H__

#include <kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <device.h>

#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/socket_offload.h>

#define MDM_MANUFACTURER_LENGTH 10
#define MDM_MODEL_LENGTH 16
#define MDM_REVISION_LENGTH 64
#define MDM_IMEI_LENGTH 16
#define MDM_TIME_LENGTH 32

struct usr_http_cfg {
	char *url;
	char *content_type;
	char *content_body;
	char *recv_buf;
	size_t recv_buf_len;
    char *resp_filename; 
	u8_t method;
	u16_t recv_read_len;
	u16_t timeout;
};

struct mdm_ctx {
	/* modem data */
	char data_manufacturer[MDM_MANUFACTURER_LENGTH];
	char data_model[MDM_MODEL_LENGTH];
	char data_revision[MDM_REVISION_LENGTH];
	char data_imei[MDM_IMEI_LENGTH];
	char data_timeval[MDM_TIME_LENGTH];
    uint32_t data_sys_timeval;
	int   data_rssi;
};

struct usr_gps_cfg {
	char *lat;
	char *lon;
    char* gps_data;
    char* agps_filename;
    char* utc_time;
    int agps_status;
};

typedef int (*mdm_get_clock)(struct device *dev,
                        char *timeval);

typedef int (*mdm_http_init)(struct device *dev,
                        struct usr_http_cfg *cfg);
typedef int (*mdm_http_execute)(struct device *dev,
                        struct usr_http_cfg *cfg);
typedef int (*mdm_http_term)(struct device *dev,
                        struct usr_http_cfg *cfg);

typedef int (*mdm_gps_init)(struct device *dev,
                        struct usr_gps_cfg *cfg);
typedef int (*mdm_gps_agps)(struct device *dev,
                        struct usr_gps_cfg *cfg);
typedef int (*mdm_gps_read)(struct device *dev,
                        struct usr_gps_cfg *cfg);
typedef int (*mdm_gps_close)(struct device *dev,
                        struct usr_gps_cfg *cfg);

typedef int (*mdm_get_ctx)(struct device *dev,
                        struct mdm_ctx *ctx);

typedef void (*mdm_reset)(void);

struct modem_quectel_bg95_net_api {
	struct net_if_api net_api;

	/* Driver specific extension api */
    mdm_get_clock  get_clock;

	mdm_http_init http_init;
	mdm_http_execute http_execute;
	mdm_http_term http_term;

	mdm_gps_init gps_init;
	mdm_gps_agps gps_agps;
	mdm_gps_read gps_read;
	mdm_gps_close gps_close;

	mdm_get_ctx get_ctx;

	mdm_reset reset;
};

enum http_method {
	HTTP_GET = 0,
	HTTP_POST,
};

__syscall int mdm_quectel_bg95_get_clock(struct device *dev,
				char *timeval);

static inline int mdm_quectel_bg95_get_clock(struct device *dev,
					   char *timeval)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->get_clock(dev, timeval);
}

__syscall int mdm_quectel_bg95_http_init(struct device *dev,
				struct usr_http_cfg *cfg);

static inline int mdm_quectel_bg95_http_init(struct device *dev,
					   struct usr_http_cfg *cfg)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->http_init(dev, cfg);
}

__syscall int mdm_quectel_bg95_http_execute(struct device *dev,
				   struct usr_http_cfg *cfg);

static inline int mdm_quectel_bg95_http_execute(struct device *dev,
					      struct usr_http_cfg *cfg)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->http_execute(dev, cfg);
}

__syscall int mdm_quectel_bg95_http_term(struct device *dev,
				struct usr_http_cfg *cfg);

static inline int mdm_quectel_bg95_http_term(struct device *dev,
					   struct usr_http_cfg *cfg)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->http_term(dev, cfg);
}

__syscall int mdm_quectel_bg95_gps_init(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_quectel_bg95_gps_init(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->gps_init(dev, cfg);
}

__syscall int mdm_quectel_bg95_gps_agps(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_quectel_bg95_gps_agps(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->gps_agps(dev, cfg);
}

__syscall int mdm_quectel_bg95_gps_read(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_quectel_bg95_gps_read(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->gps_read(dev, cfg);
}

__syscall int mdm_quectel_bg95_gps_close(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_quectel_bg95_gps_close(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->gps_close(dev, cfg);
}

__syscall int mdm_quectel_bg95_get_ctx(struct device *dev,
				struct mdm_ctx *ctx);

static inline int mdm_quectel_bg95_get_ctx(struct device *dev,
					   struct mdm_ctx *ctx)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->get_ctx(dev, ctx);
}

__syscall void mdm_quectel_bg95_reset(struct device *dev);

static inline void mdm_quectel_bg95_reset(struct device *dev)
{
	const struct modem_quectel_bg95_net_api *api =
        (const struct modem_quectel_bg95_net_api *) dev->driver_api;
	return api->reset();
}

#endif

#ifndef _MDM_A9G_RDA_H__
#define _MDM_A9G_RDA_H__

#include <kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <device.h>

#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/socket_offload.h>
#include <net/socket_offload_ops.h>

struct usr_http_cfg {
	char *url;
	char *content_type;
	char *content_body;
	char *recv_buf;
	size_t recv_buf_len;
	u8_t method;
	u16_t recv_read_len;
	u16_t timeout;
};

struct usr_gps_cfg {
	char *lat;
	char *lon;
    char* gps_data;
    int agps_status;
};

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

struct modem_a9g_rda_net_api {
	struct net_if_api net_api;

	/* Driver specific extension api */
	mdm_http_init http_init;
	mdm_http_execute http_execute;
	mdm_http_term http_term;

	mdm_gps_init gps_init;
	mdm_gps_agps gps_agps;
	mdm_gps_read gps_read;
	mdm_gps_close gps_close;
};

enum http_method {
	HTTP_GET = 0,
	HTTP_POST,
};

__syscall int mdm_a9g_http_init(struct device *dev,
				struct usr_http_cfg *cfg);

static inline int mdm_a9g_http_init(struct device *dev,
					   struct usr_http_cfg *cfg)
{
	const struct modem_a9g_rda_net_api *api =
        (const struct modem_a9g_rda_net_api *) dev->driver_api;
	return api->http_init(dev, cfg);
}

__syscall int mdm_a9g_http_execute(struct device *dev,
				   struct usr_http_cfg *cfg);

static inline int mdm_a9g_http_execute(struct device *dev,
					      struct usr_http_cfg *cfg)
{
	const struct modem_a9g_rda_net_api *api =
        (const struct modem_a9g_rda_net_api *) dev->driver_api;
	return api->http_execute(dev, cfg);
}

__syscall int mdm_a9g_http_term(struct device *dev,
				struct usr_http_cfg *cfg);

static inline int mdm_a9g_http_term(struct device *dev,
					   struct usr_http_cfg *cfg)
{
	const struct modem_a9g_rda_net_api *api =
        (const struct modem_a9g_rda_net_api *) dev->driver_api;
	return api->http_term(dev, cfg);
}

__syscall int mdm_a9g_gps_init(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_a9g_gps_init(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_a9g_rda_net_api *api =
        (const struct modem_a9g_rda_net_api *) dev->driver_api;
	return api->gps_init(dev, cfg);
}

__syscall int mdm_a9g_gps_agps(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_a9g_gps_agps(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_a9g_rda_net_api *api =
        (const struct modem_a9g_rda_net_api *) dev->driver_api;
	return api->gps_agps(dev, cfg);
}

__syscall int mdm_a9g_gps_read(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_a9g_gps_read(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_a9g_rda_net_api *api =
        (const struct modem_a9g_rda_net_api *) dev->driver_api;
	return api->gps_read(dev, cfg);
}

__syscall int mdm_a9g_gps_close(struct device *dev,
				struct usr_gps_cfg *cfg);

static inline int mdm_a9g_gps_close(struct device *dev,
					   struct usr_gps_cfg *cfg)
{
	const struct modem_a9g_rda_net_api *api =
        (const struct modem_a9g_rda_net_api *) dev->driver_api;
	return api->gps_close(dev, cfg);
}

#endif

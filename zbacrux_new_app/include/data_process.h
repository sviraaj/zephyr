#ifndef __DATA_PROCESS_H__
#define __DATA_PROCESS_H__

#include "dev_fmt.h"
#include "app_modem.h"
#include "config_params.h"

enum parse_type {
    LOGIN_DEVICE = 0,
    SEND_DATA,
    TRIP_DEVICE
};

/* 
 * The child i.e. level + 1; should come immediately after parent
 * of level.
 */
struct json_fmt_desc {
	const char *key;
	u8_t level;
    void* data;
};

int parse_server_response(struct modem_cfg *mdm_cfg,
        u8_t *data, u8_t type);
int encode_data(u8_t type, struct modem_cfg *mdm_cfg,
        u8_t* buf, u16_t buf_sz);

#endif

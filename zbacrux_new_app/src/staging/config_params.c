#if 0
#include "config_params.h"

static struct config_params cfg_params[] =
{
    [PRIMARY_SERVER]   = {PRIMARY_SERVER_OFFSET, PRIMARY_SERVER_SIZE},
    [SECONDARY_SERVER] = {SECONDARY_SERVER_OFFSET, SECONDARY_SERVER_SIZE},
    [SERVER_PORT]      = {SERVER_PORT_OFFSET, SERVER_PORT_SIZE}
    [PROVISION_URL]    = {PROVISION_URL_OFFSET, PROVISION_URL_SIZE}
    [LOGIN_URL]        = {LOGIN_URL_OFFSET, LOGIN_URL_SIZE}
    [SEND_DATA_URL]    = {SEND_DATA_URL_OFFSET, SEND_DATA_URL_SIZE}
    [DEVICE_ID]        = {DEVICE_ID_OFFSET, DEVICE_ID_SIZE}
    [SIM_ID]           = {SIM_ID_OFFSET, SIM_ID_SIZE}
    [PHONE_NUMBER]     = {PHONE_NUMBER_OFFSET, PHONE_NUMBER_SIZE}
    [UPLOAD_INTERVAL]  = {UPLOAD_INTERVAL_OFFSET, UPLOAD_INTERVAL_SIZE}
    [DATA_INTERVAL]    = {DATA_INTERVAL_OFFSET, DATA_INTERVAL_SIZE}
    [SMS_CODE]         = {SMS_CODE_OFFSET, SMS_CODE_SIZE}
    [POST_SMS_NUMBER]  = {POST_SMS_NUMBER_OFFSET, POST_SMS_NUMBER_SIZE}
};

u16_t cfg_get_offset(u8_t cfg_param_idx)
{
    return cfg_params[cfg_param_idx].offset;
}

u16_t cfg_get_len(u8_t cfg_param_idx)
{
    return cfg_params[cfg_param_idx].len;
}
#endif

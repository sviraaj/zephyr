#ifndef __CONFIG_PARAMS_H__
#define __CONFIG_PARAMS_H__

#include <zephyr.h>

#define MAX_URL_SERVER_SIZE  256U
#define MAX_DEVICE_ID_SIZE   32U
#define MAX_PH_NUM_SIZE      32U
#define MAX_SMS_CODE_SIZE    32U
#define MAX_VERSION_SIZE     32U
#define MAX_KEY_SIZE         256U

/* 
 * not more than MAX_DATA_BATCH_SIZE can be batched
 * in a single server call
 */
#define MAX_DATA_BATCH_SIZE  8U
#define MAX_BATCHED_DATA_SIZE 3200U

struct global_params
{
    const char* sys_version;
    char srv_version[MAX_VERSION_SIZE];
    char auth_key[MAX_KEY_SIZE];
    char date_time_srv[MAX_VERSION_SIZE];
    u32_t system_clock;
    char sys_datetime[MAX_VERSION_SIZE];
    char app_err_msg[MAX_KEY_SIZE];
    u8_t update_pending;
    u16_t target_temp;

    u16_t rbt_cnt;
    u16_t app_err_cnt;
    /* 
     * device flags
     * 0: reboot flag,
     * 1: app_error flag,
     * 2: 
     * rsvd
     */
    u8_t dev_flags;
};

/* all the elements in this struct need to be word aligned */
struct device_params
{
    char primary_server[MAX_URL_SERVER_SIZE];
    char secondary_server[MAX_URL_SERVER_SIZE];
    u16_t port;
    u16_t reserved_0;
    char provision_url[MAX_URL_SERVER_SIZE];
    char login_url[MAX_URL_SERVER_SIZE];
    char send_data_url[MAX_URL_SERVER_SIZE];
    char dev_id[MAX_DEVICE_ID_SIZE];
    char sim_id[MAX_DEVICE_ID_SIZE];
    char ph_num[MAX_PH_NUM_SIZE];
    u32_t upload_interval;
    u32_t data_interval;
    char sms_code[MAX_SMS_CODE_SIZE];
    char post_sms_num[MAX_PH_NUM_SIZE];
};

#if 0
struct config_params
{
    size_t offset;
    size_t len;
};

#define PRIMARY_SERVER_OFFSET        0U
#define SECONDARY_SERVER_OFFSET      256U
#define SERVER_PORT_OFFSET           260U
#define PROVISION_URL_OFFSET         516U 
#define LOGIN_URL_OFFSET             772U
#define SEND_DATA_URL_OFFSET         1028U
#define DEVICE_ID_OFFSET             1540U
#define SIM_ID_OFFSET                1572U
#define PHONE_NUMBER_OFFSET          1604U
#define UPLOAD_INTERVAL_OFFSET       1620U
#define DATA_INTERVAL_OFFSET         1622U
#define SMS_CODE_OFFSET              1624U
#define POST_SMS_NUMBER_OFFSET       1640U
#define NEXT_OFFSET                  1656U

#define UINT16_SIZE             2U

#define PRIMARY_SERVER_SIZE     MAX_URL_SERVER_SIZE
#define SECONDARY_SERVER_SIZE   MAX_URL_SERVER_SIZE
#define SERVER_PORT_SIZE        UINT16_SIZE
#define PROVISION_URL_SIZE      MAX_URL_SERVER_SIZE
#define LOGIN_URL_SIZE          MAX_URL_SERVER_SIZE
#define SEND_DATA_URL_SIZE      MAX_URL_SERVER_SIZE
#define DEVICE_ID_SIZE          MAX_DEVICE_ID_SIZE
#define SIM_ID_SIZE             MAX_DEVICE_ID_SIZE
#define PHONE_NUMBER_SIZE       MAX_PH_NUM_SIZE
#define UPLOAD_INTERVAL_SIZE    UINT16_SIZE
#define DATA_INTERVAL_SIZE      UINT16_SIZE
#define SMS_CODE_SIZE           MAX_SMS_CODE_SIZE
#define POST_SMS_NUMBER_SIZE    MAX_PH_NUM_SIZE

u16_t cfg_get_offset(u8_t cfg_param_idx);
u16_t cfg_get_len(u8_t cfg_param_idx);

#endif

enum config_param_id
{
    PRIMARY_SERVER = 0,
    SECONDARY_SERVER,
    SERVER_PORT,
    PROVISION_URL,
    LOGIN_URL,
    SEND_DATA_URL,
    DEVICE_ID,
    SIM_ID,
    PHONE_NUMBER,
    UPLOAD_INTERVAL,
    DATA_INTERVAL,
    SMS_CODE,
    POST_SMS_NUMBER,
};

/* */
int dev_settings_init();

/* data ptr needs to be word aligned */
int update_settings(u8_t cfg_param_idx, u8_t *data,
        u16_t len);

/* data ptr needs to be word aligned */
int fetch_settings(u8_t cfg_param_idx, u8_t *data,
        u16_t len);

#endif

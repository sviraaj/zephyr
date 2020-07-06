#ifndef __DEV_FMT_H__
#define __DEV_FMT_H__

#include "dev_data.h"
#include "cJSON.h"

#define FMT_AUTHORIZATION_KEY "auth_key"
#define FMT_API_VERSION "version"
#define FMT_DATE_TIME "date"

#define FMT_POST_DATA_STATUS "CS"

#define FMT_SETTINGS "ST"
#define FMT_UPDATE_REQUIRED "UP"
#define FMT_UPLOAD_INTERVAL "UI"
#define FMT_DATA_COLLECTION_INTERVAL "DCT"
#define FMT_PRIMARY_SERVER_ADDR "PS"
#define FMT_SECONDARY_SERVER_ADDR "SS"
#define FMT_LOGIN_URL "LU"
#define FMT_SEND_DATA_URL "DU"
#define FMT_PROVISION_URL "PU"
#define FMT_SMS_POST_NUMBER "SN"
#define FMT_SMS_POST_CODE "SC"

#define JSON_AP              "\\\""

#define OPEN_DEV_FMT         "{"
#define CLOSE_DEV_FMT        "}"
#define SINGLE_DEV_ENTRY(entry_name)                                \
    JSON_AP entry_name JSON_AP":"JSON_AP"%s"JSON_AP

#define SINGLE_DT_ENTRY(entry_name)                                \
    JSON_AP entry_name JSON_AP":"JSON_AP"%lu"JSON_AP

#define SINGLE_DEV_DATA_FMT                                          \
    "{"JSON_AP"T1"JSON_AP":"JSON_AP"%s"JSON_AP","                    \
     JSON_AP"T2"JSON_AP":"JSON_AP"%s"JSON_AP","                      \
     JSON_AP"HP1"JSON_AP":"JSON_AP"%s"JSON_AP","                      \
     JSON_AP"HP2"JSON_AP":"JSON_AP"%s"JSON_AP","                      \
     JSON_AP"IM1"JSON_AP":"JSON_AP"%s"JSON_AP","                     \
     JSON_AP"L1"JSON_AP":"JSON_AP"%s"JSON_AP","                      \
     JSON_AP"GP"JSON_AP":"JSON_AP"%s"JSON_AP","                      \
     JSON_AP"M"JSON_AP":"JSON_AP"%s"JSON_AP","                       \
     JSON_AP"B"JSON_AP":"JSON_AP"%s"JSON_AP","                       \
     JSON_AP"TG"JSON_AP":"JSON_AP"%s"JSON_AP"}"                      \

#define SEND_DATA_FMT                                                      \
    "{"JSON_AP"ID"JSON_AP":"JSON_AP"%s"JSON_AP","                          \
    JSON_AP"auth_key"JSON_AP":"JSON_AP"%s"JSON_AP","                       \
    JSON_AP"V"JSON_AP":"JSON_AP"%s"JSON_AP","                              \
    JSON_AP"DT"JSON_AP":"JSON_AP"%s"JSON_AP","                             \
    JSON_AP"D"JSON_AP":[%s]%s}"

#define SEND_DATA_OPT_FMT                                                  \
    ","JSON_AP"S"JSON_AP":{"JSON_AP"RBT"JSON_AP":%d,"JSON_AP"SE"JSON_AP    \
    ":{"JSON_AP"v"JSON_AP":%d,"JSON_AP"msg"JSON_AP":"JSON_AP"%s"JSON_AP    \
    "},"JSON_AP"UE"JSON_AP":%d}"

#define SINGLE_DATA_COMMON_FMT                                             \
    "{"JSON_AP"DT"JSON_AP":"JSON_AP"%s"JSON_AP"}"

#define LOGIN_FMT                                       \
    "{"JSON_AP"id"JSON_AP":"JSON_AP"%s"JSON_AP","       \
    JSON_AP"hash"JSON_AP":"JSON_AP"%s"JSON_AP","        \
    JSON_AP"version"JSON_AP":"JSON_AP"%s"JSON_AP"}"

#define TRIP_FMT                                       \
    "{"JSON_AP"ID"JSON_AP":"JSON_AP"%s"JSON_AP","      \
    JSON_AP"V"JSON_AP":"JSON_AP"%s"JSON_AP","          \
    JSON_AP"auth_key"JSON_AP":"JSON_AP"%s"JSON_AP","   \
    JSON_AP"DT"JSON_AP":"JSON_AP"%s"JSON_AP","         \
    JSON_AP"A"JSON_AP":"JSON_AP"%s"JSON_AP"}"

enum dev_fmt_idx
{
    T1 = 0,
    T2,
    HP1,
    HP2,
    IM1,
    L1,
    GP,
    M,
    B,
    TG
};

int get_dev_data_batch(u8_t* data_json, u16_t len);
int convert_data_to_json(struct device_data* data);

#endif

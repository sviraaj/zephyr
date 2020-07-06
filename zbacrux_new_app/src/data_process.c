#include <string.h>
#include "data_process.h"
#include "cJSON.h"
#include "sec_id.h"

/*
 * The child i.e. level + 1; should come immediately after parent
 * of level.
 */
static const struct json_fmt_desc login_fmt[] = { { FMT_AUTHORIZATION_KEY, 0 },
                          { FMT_API_VERSION, 0 },
                          { FMT_DATE_TIME, 0 },

                          { FMT_SETTINGS, 0 },
                          { FMT_UPDATE_REQUIRED, 1 },
                          { FMT_UPLOAD_INTERVAL, 1 },
                          { FMT_DATA_COLLECTION_INTERVAL,
                            1 },
                          { FMT_PRIMARY_SERVER_ADDR, 1 },
                          { FMT_SECONDARY_SERVER_ADDR, 1 },
                          { FMT_LOGIN_URL, 1 },
                          { FMT_SEND_DATA_URL, 1 },
                          { FMT_PROVISION_URL, 1 },
                          { FMT_SMS_POST_NUMBER, 1 },
                          { FMT_SMS_POST_CODE, 1 } };

static const struct json_fmt_desc send_data_fmt[] = {
    { FMT_POST_DATA_STATUS, 0 },

    { FMT_SETTINGS, 0 },
    { FMT_UPDATE_REQUIRED, 1 },
    { FMT_UPLOAD_INTERVAL, 1 },
    { FMT_DATA_COLLECTION_INTERVAL, 1 },
    { FMT_PRIMARY_SERVER_ADDR, 1 },
    { FMT_SECONDARY_SERVER_ADDR, 1 },
    { FMT_LOGIN_URL, 1 },
    { FMT_SEND_DATA_URL, 1 },
    { FMT_PROVISION_URL, 1 },
    { FMT_SMS_POST_NUMBER, 1 },
    { FMT_SMS_POST_CODE, 1 }

};

static char dev_data_json[MAX_BATCHED_DATA_SIZE];

int parse_server_response(struct modem_cfg *mdm_cfg, u8_t *data, u8_t type)
{
    cJSON *request_json = NULL;
    cJSON *temp = NULL;
    int ret = 0;

    info_log("%s", data);

    request_json = cJSON_Parse(data);
    if (request_json == NULL) {
        error_log("json parse failed!");
        cJSON_Delete(request_json);
        return -1;
    }

    if (type == SEND_DATA || type == TRIP_DEVICE) {
        /* response parse after sensor data upload */
        temp = cJSON_GetObjectItem(request_json, FMT_POST_DATA_STATUS);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_KEY_SIZE) {
                error_log("Max size exceeded");
                ret = -1;
                goto exit;
            } else {
                if (memcmp("success", temp->valuestring,
                       strlen(temp->valuestring)) != 0) {
                    error_log("send data server resp fail");
                    ret = -1;
                    //goto exit;
                }
            }
        } else {
            error_log("post data status not found");
            ret = -1;
            goto exit;
        }

        temp = NULL;
    } else if (type == LOGIN_DEVICE) {
        /* get each items and update global variable */
        temp = cJSON_GetObjectItem(request_json, FMT_AUTHORIZATION_KEY);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_KEY_SIZE) {
                error_log("Max size exceeded");
                ret = -1;
                goto exit;
            } else {
                strcpy(mdm_cfg->g_params->auth_key,
                       temp->valuestring);
            }
        } else {
            error_log("Authorization key not found");
            ret = -1;
            goto exit;
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_API_VERSION);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_VERSION_SIZE) {
                error_log("Max size exceeded");
                ret = -1;
                goto exit;
            } else {
                strcpy(mdm_cfg->g_params->srv_version,
                       temp->valuestring);
            }
        } else {
            error_log("API version not found");
            ret = -1;
            goto exit;
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_DATE_TIME);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_VERSION_SIZE) {
                error_log("Max size exceeded");
                ret = -1;
                goto exit;
            } else {
                strcpy(mdm_cfg->g_params->date_time_srv,
                       temp->valuestring);
                mdm_cfg->g_params->system_clock = k_cycle_get_32();
            }
        } else {
            error_log(" Server date time not found");
            ret = -1;
            goto exit;
        }

        temp = NULL;
    } else {
        error_log("Wrong type");
        ret = -1;
        goto exit;
    }

    temp = cJSON_GetObjectItem(request_json, FMT_SETTINGS);
    if (temp != NULL && cJSON_IsObject(temp)) {
        temp = cJSON_GetObjectItem(request_json, "UPDATE_REQUIRED");
        if (temp != NULL && cJSON_IsNumber(temp)) {
            /* FIXME needs to be persistentv */
            mdm_cfg->g_params->update_pending = temp->valuedouble;
        } else {
            info_log("update required not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_UPLOAD_INTERVAL);
        if (temp != NULL && cJSON_IsNumber(temp)) {
            mdm_cfg->dev_params->upload_interval =
                temp->valuedouble;
        } else {
            info_log("upload interval not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json,
                       FMT_DATA_COLLECTION_INTERVAL);
        if (temp != NULL && cJSON_IsNumber(temp)) {
            mdm_cfg->dev_params->data_interval = temp->valuedouble;
        } else {
            info_log("data interval not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_PRIMARY_SERVER_ADDR);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_URL_SERVER_SIZE) {
                error_log("Max url size exceeded");
            } else {
                memcpy(mdm_cfg->dev_params->primary_server,
                       temp->valuestring, MAX_URL_SERVER_SIZE);
            }
        } else {
            info_log("primary server addr not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json,
                       FMT_SECONDARY_SERVER_ADDR);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_URL_SERVER_SIZE) {
                error_log("Max url size exceeded");
            } else {
                memcpy(mdm_cfg->dev_params->secondary_server,
                       temp->valuestring, MAX_URL_SERVER_SIZE);
            }
        } else {
            info_log("secondary server addr not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_LOGIN_URL);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_URL_SERVER_SIZE) {
                error_log("Max url size exceeded");
            } else {
                memcpy(mdm_cfg->dev_params->login_url,
                       temp->valuestring, MAX_URL_SERVER_SIZE);
            }
        } else {
            info_log("login_url not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_SEND_DATA_URL);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_URL_SERVER_SIZE) {
                error_log("Max url size exceeded");
            } else {
                memcpy(mdm_cfg->dev_params->send_data_url,
                       temp->valuestring, MAX_URL_SERVER_SIZE);
            }
        } else {
            info_log("send data url not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_PROVISION_URL);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_URL_SERVER_SIZE) {
                error_log("Max url size exceeded");
            } else {
                memcpy(mdm_cfg->dev_params->provision_url,
                       temp->valuestring, MAX_URL_SERVER_SIZE);
            }
        } else {
            info_log("provision url not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_SMS_POST_NUMBER);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_PH_NUM_SIZE) {
                error_log("Max ph num size exceeded");
            } else {
                memcpy(mdm_cfg->dev_params->ph_num,
                       temp->valuestring, MAX_PH_NUM_SIZE);
            }
        } else {
            info_log("ph num not found");
        }

        temp = NULL;

        temp = cJSON_GetObjectItem(request_json, FMT_SMS_POST_CODE);
        if (temp != NULL && cJSON_IsString(temp)) {
            if (strlen(temp->valuestring) > MAX_SMS_CODE_SIZE) {
                error_log("Max sms code sz exceeded");
            } else {
                memcpy(mdm_cfg->dev_params->sms_code,
                       temp->valuestring, MAX_SMS_CODE_SIZE);
            }
        } else {
            info_log("provision url not found");
        }

        temp = NULL;

        /* TODO Semaphore for commiting the dev settings to flash */
        k_sem_give(&mdm_cfg->serv_resp_sem);
    } else {
        if (type == LOGIN_DEVICE) {
            error_log("No settings given");
            ret = -1;
        } else if(type == SEND_DATA) {
            info_log("No settings given send data");
            //ret = 0; /* No error */
        }
    }

exit:
    /* End of JSON parsing */
    cJSON_Delete(request_json);

    return ret;
}

int encode_data(u8_t type, struct modem_cfg *mdm_cfg, u8_t *buf, u16_t buf_sz)
{
    int ret = 0;
    char hash[MAX_DEVICE_ID_SIZE * 2] = {0};

    if (type == SEND_DATA) {
        ret = get_dev_data_batch(dev_data_json, MAX_BATCHED_DATA_SIZE);

        /* check if we need to send status */
        if (mdm_cfg->g_params->dev_flags != 0) {
            char opt_status_fmt[512];
            snprintf(opt_status_fmt, 512, SEND_DATA_OPT_FMT,
                 mdm_cfg->g_params->rbt_cnt,
                 mdm_cfg->g_params->app_err_cnt,
                 mdm_cfg->g_params->app_err_msg,
                 mdm_cfg->g_params->update_pending);

            snprintf(buf, buf_sz, SEND_DATA_FMT,
                 mdm_cfg->dev_params->dev_id,
                 mdm_cfg->g_params->auth_key,
                 mdm_cfg->g_params->sys_version,
                 mdm_cfg->g_params->sys_datetime,
                 dev_data_json, opt_status_fmt);
        } else {
            snprintf(buf, buf_sz, SEND_DATA_FMT,
                 mdm_cfg->dev_params->dev_id,
                 mdm_cfg->g_params->auth_key,
                 mdm_cfg->g_params->sys_version,
                 mdm_cfg->g_params->sys_datetime,
                 dev_data_json, "");
        }
    } else if (type == LOGIN_DEVICE) {
        mutate_id_secret(mdm_cfg->dev_params->dev_id, hash);
        snprintf(buf, buf_sz, LOGIN_FMT, mdm_cfg->dev_params->dev_id,
             hash, mdm_cfg->g_params->sys_version);
    } else if (type == TRIP_DEVICE) {
        snprintf(buf, buf_sz, TRIP_FMT, mdm_cfg->dev_params->dev_id,
             mdm_cfg->g_params->sys_version, mdm_cfg->g_params->auth_key,
             mdm_cfg->g_params->sys_datetime, mdm_cfg->g_params->trip_status);
    }

    return 0;
}

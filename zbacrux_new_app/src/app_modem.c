#include <zephyr.h>
#include <device.h>
#include <kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <net/socket.h>
#include <drivers/modem/a9grda.h>
#include "app_modem.h"
#include "data_process.h"
#include "app_log.h"

char s_url[MAX_URL_SIZE];
char s_body_content[MAX_BODY_SIZE];
char s_content_type[MAX_MISC_SIZE];
char http_recv_buf[MAX_RECV_RESP_BUF] = { 0 };

int login_device(struct device* modem_dev, struct modem_cfg *mdm_cfg)
{
	int status = 0;
	struct usr_http_cfg cfg = {
		.method = HTTP_POST,
		.timeout = 50000,
	};

	/* Post Data */
	memset(s_url, 0, sizeof(s_url));
	memset(s_body_content, 0, sizeof(s_body_content));
	memset(s_content_type, 0, sizeof(s_content_type));
	memset(http_recv_buf, 0, MAX_RECV_RESP_BUF);

	snprintf(s_url, sizeof(s_url), "%s%s", mdm_cfg->dev_params->primary_server,
		 mdm_cfg->dev_params->login_url);
	snprintf(s_content_type, sizeof(s_content_type), "%s",
		 "application/json");

    /* Frame baody for login */
    encode_data(LOGIN_DEVICE, mdm_cfg, s_body_content, sizeof(s_body_content));
	//snprintf(s_body_content, sizeof(s_body_content), LOGIN_CONTEXT, nImei,
	//	 hash(nImei), mdm_cfg->version);

	cfg.url = s_url;
	cfg.content_type = s_content_type;
	cfg.content_body = s_body_content;
	cfg.recv_buf = http_recv_buf;
	cfg.recv_buf_len = MAX_RECV_RESP_BUF;

	info_log("Login device %s", s_url);

	status = mdm_a9g_http_init(modem_dev, &cfg);
	if (status != 0) {
		error_log("mdm_a9g_http_init fail.");
		return status;
	}

	status = mdm_a9g_http_execute(modem_dev, &cfg);
	if (status != 0) {
		error_log("mdm_a9g_http_execute fail.");
		return status;
	}

    info_log("login response received");

    status = mdm_a9g_http_term(modem_dev, &cfg);
    if (status != 0)
    {
        error_log("mdm_a9g_http_term fail.");
        return status;
    }

	/* Parse response from server */
	status = parse_server_response(mdm_cfg, http_recv_buf,
            LOGIN_DEVICE);
	if (status != 0) {
		error_log("parse_server_response fail.");
		return status;
	}

	return status;
}

int send_device_data(struct device* modem_dev, struct modem_cfg *mdm_cfg)
{
	int status = 0;
	struct usr_http_cfg cfg = {
		.method = HTTP_POST,
		.timeout = 50000,
	};

	/* Post Data */
	memset(s_url, 0, sizeof(s_url));
	memset(s_body_content, 0, sizeof(s_body_content));
	memset(s_content_type, 0, sizeof(s_content_type));
	memset(http_recv_buf, 0, MAX_RECV_RESP_BUF);

	snprintf(s_url, sizeof(s_url), "%s%s", mdm_cfg->dev_params->primary_server,
		 mdm_cfg->dev_params->send_data_url);
	snprintf(s_content_type, sizeof(s_content_type), "%s",
		 "application/json");

    encode_data(SEND_DATA, mdm_cfg, s_body_content, sizeof(s_body_content));

    /*
	snprintf(
		s_body_content, sizeof(s_body_content),
		"{\\\"RFDeviceId\\\":\\\"%s\\\",\\\"coordinates\\\": [%f,%f],"
		"\\\"CoordinatesTime\\\":\\\"%s\\\",\\\"speed\\\":\\\"%f\\\",\\\"sat_"
		"tracked\\\":\\\"%d\\\",\\\"gps_fix_type\\\":\\\"%d\\\","
		"\\\"temperature\\\":\\\"%"
		"f\\\",\\\"humidity\\\":\\\"%f\\\",\\\"bat_v\\\":\\\"%d\\\",\\\"bat_p\\\":"
		"\\\"%d\\\"}",
		s_data->dev_id, s_data->lat, s_data->lon, s_data->timeStr,
		s_data->speed, s_data->sat_tk, s_data->is_fixed,
		s_data->temperature, s_data->humidity, s_data->battery_voltage,
		s_data->battery_percent);
    */

	cfg.url = s_url;
	cfg.content_type = s_content_type;
	cfg.content_body = s_body_content;
	cfg.recv_buf = http_recv_buf;
	cfg.recv_buf_len = MAX_RECV_RESP_BUF;

	info_log("Device data send %s", s_url);

#if 0
    info_log("content %d", strlen(s_body_content));
    for (int i = 0; i < strlen(s_body_content); i += 128)
    {
        info_log("%s", s_body_content + (i * 128));
        k_sleep(10);
    }
#endif

	status = mdm_a9g_http_init(modem_dev, &cfg);
	if (status != 0) {
		error_log("mdm_a9g_http_init fail.");
		return status;
	}

	status = mdm_a9g_http_execute(modem_dev, &cfg);
	if (status != 0) {
		error_log("mdm_a9g_http_execute fail.");
		return status;
	}

    info_log("send data response received");

    status = mdm_a9g_http_term(modem_dev, &cfg);
    if (status != 0)
    {
        error_log("mdm_a9g_http_term fail.");
        return status;
    }

	/* Parse response from server */
	status = parse_server_response(mdm_cfg, http_recv_buf,
            SEND_DATA);
	if (status != 0) {
		error_log("parse_server_response fail.");
		return status;
	}

	return status;
}

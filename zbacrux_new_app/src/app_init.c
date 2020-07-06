#include "app_cfg.h"
#include "app_compressor.h"
#include "cJSON.h"
#include "dev_data.h"
#include "app_io.h"
#include <string.h>

#define MAX_INT_NEG_32    0xf0000000

static cJSON_Hooks zephyr_hooks = {
    k_malloc,
    k_free
};

int app_init(struct app_config *app_cfg)
{
    struct sensor_dev sensor_devs;

    int ret = 0;

    /* general init */
    cJSON_InitHooks(&zephyr_hooks);

    app_cfg->modem_dev = device_get_binding(DT_INST_0_RDA_A9G_LABEL);
    app_cfg->imu_dev = device_get_binding(DT_INST_0_INVENSENSE_ICM20XXX_LABEL);
    app_cfg->flash_dev = device_get_binding(DT_INST_0_QSPI_MX25RXX_LABEL);
    app_cfg->gpio0_dev = device_get_binding(DT_ALIAS_GPIO_0_LABEL);
    app_cfg->gpio1_dev = device_get_binding(DT_ALIAS_GPIO_1_LABEL);
    app_cfg->gpio_p0_dev = device_get_binding(DT_INST_0_SEMTECH_SX1509B_LABEL);
    app_cfg->uart1_dev = device_get_binding(DT_ALIAS_UART_1_LABEL);
    app_cfg->pwm_dev = device_get_binding(DT_ALIAS_PWM_0_LABEL);
    app_cfg->display_dev = device_get_binding(DT_SPI_3_NAME);
    app_cfg->env1_dev = device_get_binding(DT_INST_1_BOSCH_BME280_LABEL);
    app_cfg->env2_dev = device_get_binding(DT_INST_0_BOSCH_BME280_LABEL);

    app_cfg->mdm_cfg.dev_params = &app_cfg->dev_params;
    app_cfg->mdm_cfg.g_params = &app_cfg->g_params;

    /* fetch settings from storage */
    ret = dev_settings_init();
    if (ret != 0)
    {
        error_log("dev settings init failed");
    }

    fetch_settings(PRIMARY_SERVER, app_cfg->dev_params.primary_server,
            MAX_URL_SERVER_SIZE);
    /* FIXME  Hack */
    strcpy(app_cfg->dev_params.primary_server, "https://ec2-54-187-232-160.us-west-2.compute.amazonaws.com:3000");
    fetch_settings(SECONDARY_SERVER, app_cfg->dev_params.secondary_server,
            MAX_URL_SERVER_SIZE);
    fetch_settings(SERVER_PORT, (u8_t *) &app_cfg->dev_params.port,
            sizeof(u32_t));
    fetch_settings(PROVISION_URL, app_cfg->dev_params.provision_url,
            MAX_URL_SERVER_SIZE);
    fetch_settings(LOGIN_URL, app_cfg->dev_params.login_url,
            MAX_URL_SERVER_SIZE);
    fetch_settings(SEND_DATA_URL, app_cfg->dev_params.send_data_url,
            MAX_URL_SERVER_SIZE);
    fetch_settings(DEVICE_ID, app_cfg->dev_params.dev_id,
            MAX_DEVICE_ID_SIZE);
    fetch_settings(SIM_ID, app_cfg->dev_params.sim_id,
            MAX_DEVICE_ID_SIZE);
    fetch_settings(PHONE_NUMBER, app_cfg->dev_params.ph_num,
            MAX_PH_NUM_SIZE);
    fetch_settings(UPLOAD_INTERVAL, (u8_t *) &app_cfg->dev_params.upload_interval,
            sizeof(u32_t));
    fetch_settings(DATA_INTERVAL, (u8_t *) &app_cfg->dev_params.data_interval,
            sizeof(u32_t));
    fetch_settings(SMS_CODE, app_cfg->dev_params.sms_code,
            MAX_SMS_CODE_SIZE);
    fetch_settings(POST_SMS_NUMBER, app_cfg->dev_params.post_sms_num,
            MAX_PH_NUM_SIZE);

    info_log("%s", app_cfg->dev_params.primary_server);
    info_log("%s", app_cfg->dev_params.secondary_server);
    info_log("%s", app_cfg->dev_params.provision_url);
    info_log("%s", app_cfg->dev_params.login_url);
    info_log("%s", app_cfg->dev_params.send_data_url);
    info_log("%s", app_cfg->dev_params.dev_id);
    info_log("%s", app_cfg->dev_params.sim_id);
    info_log("%s", app_cfg->dev_params.ph_num);
    info_log("%d", app_cfg->dev_params.upload_interval);
    info_log("%d", app_cfg->dev_params.data_interval);
    info_log("%s", app_cfg->dev_params.sms_code);
    info_log("%s", app_cfg->dev_params.post_sms_num);

    /* FIXME display init */
    //ret = display_init(display_dev, gpio0_dev);

    /* io init */
    ret = app_io_init(app_cfg->gpio_p0_dev, app_cfg->pwm_dev);

    /* */
    sensor_devs.env1_dev = app_cfg->env1_dev;
    sensor_devs.env2_dev = app_cfg->env2_dev;
    sensor_devs.imu_dev = app_cfg->imu_dev;
    sensor_devs.gps_dev = app_cfg->modem_dev;

    info_log("Init dev_data");

    /* dev data init */
    ret = dev_data_init(&sensor_devs);

    info_log("Init compressor");

    /* control systems init */
    ret = compressor_init(app_cfg->uart1_dev, app_cfg->gpio_p0_dev);

    /* DO NOT start compressor until instructed */
    //compressor_set_target_temp(MAX_INT_NEG_32);
    /* FIXME Testing tgt temp 5 */
    compressor_set_target_temp(500);

    return ret;
}

#if 0

	gpio_pin_configure(gpio, 11, GPIO_DIR_OUT);
    gpio_pin_write(gpio, 11, 1);

	gpio_pin_configure(gpio, 0, GPIO_DIR_OUT);
    gpio_pin_write(gpio, 0, 0);
    k_sleep(1000);
    gpio_pin_write(gpio, 0, 1);

	pwm = device_get_binding("PWM_0");

#if 0
	k_work_init(&beep_work, beep);
	/* Notify with a beep that we've started */
	k_work_submit(&beep_work);
#endif
    u32_t period, min_period, max_period;
    u64_t cycles;

	/* Adjust max_period depending pwm capabilities */
	pwm_get_cycles_per_sec(pwm, BUZZER_PIN, &cycles);
	/* in very worst case, PWM has a 8 bit resolution and count up to 256 */
	period = max_period = 256 * USEC_PER_SEC / cycles;
	min_period = max_period / 64;

	pwm_pin_set_usec(pwm, BUZZER_PIN, 250, 125);
    k_sleep(5000);
	pwm_pin_set_usec(pwm, BUZZER_PIN, 124, 62);
    k_sleep(5000);
	pwm_pin_set_usec(pwm, BUZZER_PIN, 62, 31);
    k_sleep(5000);
	pwm_pin_set_usec(pwm, BUZZER_PIN, 30, 15);
    k_sleep(5000);
#endif

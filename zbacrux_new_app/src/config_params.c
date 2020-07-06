/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config_params.h"
#include <drivers/flash.h>
#include <device.h>
#include <fs/nvs.h>   
#include "app_log.h"

static struct nvs_fs fs;

int dev_settings_init()
{
    int rc = 0;
    struct flash_pages_info info;

    /* 
     * define the nvs file system by settings with:
     * sector_size equal to the pagesize,
     * starting at DT_FLASH_AREA_STORAGE_OFFSET
     */
    fs.offset = DT_FLASH_AREA_SETTINGS_STORE_OFFSET;
    rc = flash_get_page_info_by_offs(
            device_get_binding(DT_FLASH_AREA_SETTINGS_STORE_DEV),
            fs.offset, &info);
    if (rc != 0) {
        error_log("Unable to get page info");
    }
    fs.sector_size = info.size;
    fs.sector_count = DT_FLASH_AREA_SETTINGS_STORE_SIZE / fs.sector_size;

    rc = nvs_init(&fs, DT_FLASH_AREA_SETTINGS_STORE_DEV);
    if (rc != 0) {
        error_log("Flash settings Init failed\n");
    }

    return rc;
}

int update_settings(u8_t cfg_param_idx, u8_t *data,
        u16_t len)
{
    return nvs_write(&fs, (u8_t) cfg_param_idx, data, len);
}

int fetch_settings(u8_t cfg_param_idx, u8_t *data,
        u16_t len)
{
    return nvs_read(&fs, (u8_t) cfg_param_idx, data, len);
}

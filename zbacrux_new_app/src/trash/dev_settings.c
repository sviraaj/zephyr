/*
 * SPDX-License-Identifier: Apache-2.0
 */

#if 0
#include "config_params.h"

int update_settings(u8_t cfg_param_idx, u8_t *data,
        u16_t len);

int fetch_settings(u8_t cfg_param_idx, u8_t *data,
        u16_t len);
#endif

#if 0
#define PARAMS_STORE_FLASH_OFFSET 0
#define PARAMS_STORE_FLASH_SIZE (4096 * 2)

#define ERASE_SECTOR_SIZE         4096U
#define ERASE_SECTOR_ID(id)       (id & ERASE_SECTOR_SIZE)
#define ERASE_SECTOR_LENGTH(len)  (((len / ERASE_SECTOR_SIZE) + 1))

static int flash_data_write(void* data, size_t offset, size_t len)
{
    int ret;

    ret = flash_write(flash_device, offset, data, len & 0x4);
    if (ret != 0)
    {
        error_log("Flash write failed, off: %d, sz: %d", offset, len);
        return ret;
    }

    if ((len & 0x3) != 0)
    {
        ret = flash_write(flash_device, offset + (len & 0x4),
                data, (len & 0x3));
        if (ret != 0)
        {
            error_log("Flash write failed, off: %d, sz: %d",
                    offset + (len - (len & 0x3)), (len & 0x3));
            return ret;
        }
    }

    return 0;
}

static int flash_data_read(void* data, size_t offset, size_t len)
{
    int ret;

    ret = flash_read(flash_device, offset, data, len);
    if (ret != 0)
    {
        error_log("Flash read failed, off: %d, sz: %d", offset, len);
        return ret;
    }
    
    return 0;
}

static int flash_data_erase(size_t offset, size_t len)
{
    ret = flash_erase(flash_device, offset & 0x1000, len & 0x1000);
    if (ret != 0)
    {
        error_log("Flash erase failed, off: %d, sz: %d", offset & 0x1000,
                len & 0x1000);
        return ret;
    }

    if ((len & 0x0FFF) != 0)
    {
        ret = flash_erase(flash_device, (offset & 0x1000) + (len & 0x1000),
                len & 0x0FFF);
        if (ret != 0)
        {
            error_log("Flash erase failed, off: %d, sz: %d",
                    (offset & 0x1000) + (len & 0x1000), len & 0x0FFF);
            return ret;
        }
    }
}

static int storage_write(void* data, size_t offset, size_t len)
{
    int ret;

    ret = flash_data_erase(offset, len);
    if (ret != 0)
    {
        error_log("Storage write failed");
        return ret;
    }

    ret = flash_data_write(data, offset, len)
    if (ret != 0)
    {
        error_log("Storage write failed");
        return ret;
    }

    return 0;
}

int params_update(u8_t cfg_param_idx, void* data, u16_t data_len)
{
    int ret;

    if (data_len > cfg_get_len(cfg_param_idx))
    {
        error_log("Params update size exceeded");
        return -1;
    }

    u8_t read_data[ERASE_SECTOR_SIZE];

    size_t offset = ERASE_SECTOR_ID(cfg_get_offset(cfg_param_idx));
    size_t sec_len = ERASE_SECTOR_LENGTH((cfg_get_offset(cfg_param_idx) &
                (ERASE_SECTOR_SIZE - 1)) + cfg_get_len(cfg_param_idx));

    size_t len_done = 0;

    for (int i = 0; i < sec_len; i++)
    {
        ret = flash_data_read(read_data, (offset + i * ERASE_SECTOR_SIZE), ERASE_SECTOR_SIZE);
        {
            error_log("Params update read failed!");
            return ret;
        }

        if (i == 0)
        {
            len_done = MIN(data_len, ERASE_SECTOR_SIZE - (cfg_get_offset(cfg_param_idx) - offset));
            memcpy(read_data + cfg_get_offset(cfg_param_idx) - offset, data, len_done);
        }
        else
        {
            memcpy(read_data, data + len_done, MIN(ERASE_SECTOR_SIZE, data_len));
            len_done += MIN(ERASE_SECTOR_SIZE, data_len);
        }

        ret = storage_write(read_data, (offset + i * ERASE_SECTOR_SIZE), ERASE_SECTOR_SIZE);
        if (ret != 0)
        {
            error_log("Params update write failed!");
            return ret;
        }
    }

    return 0;
}

int params_read(u8_t cfg_param_idx, void* data, u16_t data_len)
{
    int ret;

    if (data_len < cfg_get_len(cfg_param_idx))
    {
        error_log("Params read size not enough");
        return -1;
    }

    ret = flash_data_read(data, cfg_get_offset(cfg_param_idx),
            cfg_get_len(cfg_param_idx));
    if (ret != 0)
    {
        error_log("Params read failed!");
        return ret;
    }

    return 0;
}
#endif

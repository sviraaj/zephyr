/*
 * Copyright (c) 2019 ZedBlox Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reference:
 * nrf5_sdk/components/libraries/block_dev/qspi
 */

#include <errno.h>

#include <drivers/flash.h>
#include <init.h>
#include <string.h>
#include "flash_priv.h"
#include "qspi_flash_mx25rxx.h"

#define LOG_DOMAIN "qspi_flash_mx25rxx"
#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(qspi_flash_mx25rxx);

#if defined(CONFIG_MULTITHREADING)
#define SYNC_INIT()                                                            \
	k_sem_init(&((struct qspi_flash_data *)dev->driver_data)->sem, 1,      \
		   UINT_MAX)
#define SYNC_LOCK() k_sem_take(&driver_data->sem, K_FOREVER)
#define SYNC_UNLOCK() k_sem_give(&driver_data->sem)
#else
#define SYNC_INIT()
#define SYNC_LOCK()
#define SYNC_UNLOCK()
#endif

#define OP_INIT()                                                              \
	k_sem_init(&((struct qspi_flash_data *)dev->driver_data)->sem_op, 0,   \
		   UINT_MAX)
#define OP_LOCK() k_sem_take(&driver_data->sem_op, K_FOREVER)
#define OP_UNLOCK() k_sem_give(&driver_data->sem_op)

#define QSPI_STD_CMD_WRSR 0x01 /**< Write status register command*/
#define QSPI_STD_CMD_RSTEN 0x66 /**< Reset enable command*/
#define QSPI_STD_CMD_RST 0x99 /**< Reset command*/
#define QSPI_STD_CMD_READ_ID 0x9F /**< Read ID command*/

#define QSPI_FLASH_PAGE_PROGRAM_SIZE  256 /**< Page program size (minimum block size)*/

#define BD_ERASE_UNIT_INVALID_ID 0xFFFFFFFF /**< Invalid erase unit number*/
#define BD_ERASE_UNIT_ERASE_VAL 0xFFFFFFFF /**< Erased memory value*/

#define QSPI_ERASE_SECTOR_SIZE 4096
#define QSPI_ERASE_BLOCK32K_SIZE (32 * 1024)
#define QSPI_ERASE_BLOCK_SIZE (64 * 1024)
#define QSPI_BLOCK_COUNT 128
#define QSPI_FLASH_SECTOR_MASK (0xFFF)
#define QSPI_FLASH_CHIP_SIZE (8 * 1024 * 1024)

/**
 * @brief Block to erase unit translation
 *
 * @param blk_id    Block index
 * @param blk_size  Block size
 * */
#define BD_BLOCK_TO_ERASEUNIT(blk_id, blk_size)                                \
	((blk_id) * (blk_size)) / (NRF_BLOCK_DEV_QSPI_ERASE_UNIT_SIZE)

/**
 * @brief Blocks per erase unit
 *
 * @param blk_size  Block size
 * */
#define BD_BLOCKS_PER_ERASEUNIT(blk_size)                                      \
	(NRF_BLOCK_DEV_QSPI_ERASE_UNIT_SIZE / (blk_size))

#define QSPI_BLOCK_SIZE 4U
#define WORD_ALIGNED(x)     ((u32_t) (((u32_t)x) & (QSPI_BLOCK_SIZE - 1)))
#define IS_WORD_ALIGNED(x)  ((u8_t) (WORD_ALIGNED(x) == 0))

enum work_state {
	NRF_BLOCK_DEV_QSPI_STATE_IDLE = 0,
	NRF_BLOCK_DEV_QSPI_STATE_READ_EXEC,
	NRF_BLOCK_DEV_QSPI_STATE_WRITE_ERASE,
	NRF_BLOCK_DEV_QSPI_STATE_PAGE_LOAD,
};

static int p_work_state = NRF_BLOCK_DEV_QSPI_STATE_IDLE;

static void qspi_handler(nrfx_qspi_evt_t event, void *p_context)
{
	struct qspi_flash_data *driver_data =
		((struct device *)p_context)->driver_data;

	switch (p_work_state) {
	case NRF_BLOCK_DEV_QSPI_STATE_READ_EXEC: {
		p_work_state = NRF_BLOCK_DEV_QSPI_STATE_IDLE;

		OP_UNLOCK();

		break;
	}
	case NRF_BLOCK_DEV_QSPI_STATE_WRITE_ERASE:
	case NRF_BLOCK_DEV_QSPI_STATE_PAGE_LOAD: {
		p_work_state = NRF_BLOCK_DEV_QSPI_STATE_IDLE;

		OP_UNLOCK();

		break;
	}

	default:
		break;
	}
}

#if 0
static int block_dev_qspi_uninit(struct device *dev)
{
	nrfx_qspi_uninit();

	return 0;
}
#endif

static int block_dev_qspi_read_req(struct device *dev, off_t offset, void *data,
				   size_t len)
{
	int ret;
    u8_t __aligned(4) block_buf[QSPI_BLOCK_SIZE];
    u16_t rd_len_rem = len;
    u16_t t_rd_len = 0;
    u8_t* rd_data = (u8_t *) data;
    u16_t pre_rd_len = 0;

	struct qspi_flash_data *driver_data = dev->driver_data;

    if (IS_WORD_ALIGNED(data) != 1)
    {
        pre_rd_len = (QSPI_BLOCK_SIZE - ((u32_t)rd_data & (QSPI_BLOCK_SIZE - 1)));
        rd_data += pre_rd_len;
        rd_len_rem -= pre_rd_len;
    }

	SYNC_LOCK();

    if (rd_data != data)
    {
        p_work_state = NRF_BLOCK_DEV_QSPI_STATE_READ_EXEC;

        ret = nrfx_qspi_read(block_buf, QSPI_BLOCK_SIZE, offset);
        if (ret != NRFX_SUCCESS) {
            LOG_ERR("QSPI flash read fail, 0x%x, len: %d", ret, rd_len_rem);
            ret = -EIO;
            goto exit;
        }

        OP_LOCK();

        offset += pre_rd_len;

        memcpy((u8_t* )data, block_buf, pre_rd_len);
    }

    t_rd_len = (rd_len_rem & ~(QSPI_BLOCK_SIZE - 1));

	p_work_state = NRF_BLOCK_DEV_QSPI_STATE_READ_EXEC;

	ret = nrfx_qspi_read(rd_data, t_rd_len, offset);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("QSPI flash read fail, 0x%x, len: %d", ret, rd_len_rem);
		ret = -EIO;
		goto exit;
	}

    rd_len_rem -= t_rd_len;

	OP_LOCK();

    if (rd_len_rem != 0)
    {
        p_work_state = NRF_BLOCK_DEV_QSPI_STATE_READ_EXEC;

        ret = nrfx_qspi_read(block_buf, QSPI_BLOCK_SIZE, offset + t_rd_len);
        if (ret != NRFX_SUCCESS) {
            LOG_ERR("QSPI flash read fail, 0x%x, len: %d", ret, QSPI_BLOCK_SIZE);
            ret = -EIO;
            goto exit;
        }

        OP_LOCK();

        memcpy(((u8_t* )rd_data) + t_rd_len, block_buf,  rd_len_rem);
    }

    ret = 0;

exit:
	SYNC_UNLOCK();

	return ret;
}

static int block_dev_qspi_erase_internal(struct device *dev, off_t offset,
					 size_t size)
{
	struct qspi_flash_data *const driver_data = dev->driver_data;
	int ret;

    nrf_qspi_erase_len_t length = NRF_QSPI_ERASE_LEN_4KB;

    if (size == QSPI_FLASH_CHIP_SIZE)
    {
        length = NRF_QSPI_ERASE_LEN_ALL;
    }
    else if (size == QSPI_ERASE_BLOCK_SIZE)
    {
        length = NRF_QSPI_ERASE_LEN_64KB;
    }
    else if (size == QSPI_ERASE_SECTOR_SIZE)
    {
        length = NRF_QSPI_ERASE_LEN_4KB;
    }
    else
    {
        LOG_ERR("QSPI erase invalid size");
        ret = -EINVAL;
        goto exit;
    }

	p_work_state = NRF_BLOCK_DEV_QSPI_STATE_WRITE_ERASE;

    ret = nrfx_qspi_erase(length, offset);
    if (ret != NRFX_SUCCESS) {
        LOG_ERR("QSPI erase fail 0x%x", ret);
        ret = -EIO;
        goto exit;
    }

    ret = 0;

	OP_LOCK();

exit:
	return ret;
}

static int block_dev_qspi_erase(struct device *dev, off_t offset, size_t size)
{
	struct qspi_flash_data *const driver_data = dev->driver_data;
	int ret = 0;
	u32_t new_offset = offset;
	u32_t size_remaining = size;

	if ((offset < 0) || ((offset & QSPI_FLASH_SECTOR_MASK) != 0) ||
	    ((size + offset) > QSPI_FLASH_CHIP_SIZE) ||
	    ((size & QSPI_FLASH_SECTOR_MASK) != 0)) {
        LOG_ERR("Invalid params");
		return -ENODEV;
	}

	SYNC_LOCK();

    if (size == QSPI_FLASH_CHIP_SIZE)
    {
        ret = block_dev_qspi_erase_internal(dev, new_offset, QSPI_FLASH_CHIP_SIZE);
		if (ret != 0) {
			LOG_ERR("Erase 0x%x size fail", QSPI_FLASH_CHIP_SIZE);
			ret = -EIO;
        }
        goto exit;
    }

	while (size_remaining >= QSPI_ERASE_BLOCK_SIZE) {
		ret = block_dev_qspi_erase_internal(dev, new_offset,
						    QSPI_ERASE_BLOCK_SIZE);
		if (ret != 0) {
			LOG_ERR("Erase 0x%x size fail", QSPI_ERASE_BLOCK_SIZE);
			ret = -EIO;
			goto exit;
		}

		new_offset += QSPI_ERASE_BLOCK_SIZE;
		size_remaining -= QSPI_ERASE_BLOCK_SIZE;
	}

	while (size_remaining >= QSPI_ERASE_SECTOR_SIZE) {
		ret = block_dev_qspi_erase_internal(dev, new_offset,
						    QSPI_ERASE_SECTOR_SIZE);
		if (ret != 0) {
			LOG_ERR("Erase 0x%x size fail", QSPI_ERASE_SECTOR_SIZE);
			ret = -EIO;
			goto exit;
		}

		new_offset += QSPI_ERASE_SECTOR_SIZE;
		size_remaining -= QSPI_ERASE_SECTOR_SIZE;
	}

exit:
	SYNC_UNLOCK();

	return ret;
}

static int block_dev_qspi_program_page(struct device *dev, off_t offset,
				       const void *data, size_t len)
{
	struct qspi_flash_data *driver_data = dev->driver_data;
	int ret;

	p_work_state = NRF_BLOCK_DEV_QSPI_STATE_PAGE_LOAD;

    if (len <= 0)
    {
        return -EINVAL;
    }

	ret = nrfx_qspi_write(data, len, offset);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("QSPI page program fail 0x%x", ret);
		ret = -EIO;
		goto exit;
	}

    ret = 0;

	OP_LOCK();

exit:
	return ret;
}

static int block_dev_qspi_write_req(struct device *dev, off_t offset,
				    const void *data, size_t len)
{
	struct qspi_flash_data *driver_data = dev->driver_data;
	int ret;
	off_t page_offset;
	/* Cast `data`  to prevent `void*` arithmetic */
	const u8_t *data_ptr = data;

    if (IS_WORD_ALIGNED(data) != 1)
    {
		LOG_ERR("alignment failed write data buf");
        return -EINVAL;
    }

    if (IS_WORD_ALIGNED(len) != 1)
    {
		LOG_ERR("alignment failed write buf len");
        return -EINVAL;
    }

	if (offset < 0) {
		return -ENOTSUP;
	}

	SYNC_LOCK();

	/* Calculate the offset in the first page we write */
	page_offset = offset % QSPI_FLASH_PAGE_PROGRAM_SIZE;

	/*
	 * Write all data that does not fit into a single programmable page.
	 * By doing this logic, we can safely disable lock protection in
	 * between pages as in case the user did not disable protection then
	 * it will fail on the first write.
	 */
	while ((page_offset + len) > QSPI_FLASH_PAGE_PROGRAM_SIZE) {
		size_t len_to_write_in_page =
			QSPI_FLASH_PAGE_PROGRAM_SIZE - page_offset;

		ret = block_dev_qspi_program_page(dev, offset, data_ptr,
						  len_to_write_in_page);
		if (ret) {
			goto end;
		}

		len -= len_to_write_in_page;
		offset += len_to_write_in_page;
		data_ptr += len_to_write_in_page;

		/*
		 * For the subsequent pages we always start at the beginning
		 * of a page
		 */
		page_offset = 0;
	}

    if (len > 0)
    {
        ret = block_dev_qspi_program_page(dev, offset, data_ptr, len);
    }

end:
	SYNC_UNLOCK();

	return ret;
}

static int block_dev_write_protection_set(struct device *dev, bool write_protect)
{
    /* TODO Need implmentation? */
    return 0;
}

static int block_dev_qspi_init(struct device *dev)
{
	nrfx_err_t ret;
	uint8_t temporary = 0x40;

	struct qspi_flash_data *driver_data = dev->driver_data;

	SYNC_INIT();
	OP_INIT();

	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_QSPI),
		    6, nrfx_isr, nrfx_qspi_irq_handler, 0);

	nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG(19, 18, 22, 23, 32, 21);

	memcpy(&driver_data->qspi_cfg, &config, sizeof(nrfx_qspi_config_t));

	driver_data->qspi_cfg.pins.sck_pin = 19;
	driver_data->qspi_cfg.pins.csn_pin = 18;
	driver_data->qspi_cfg.pins.io0_pin = 22;
	driver_data->qspi_cfg.pins.io1_pin = 23;
	driver_data->qspi_cfg.pins.io2_pin = 32;
	driver_data->qspi_cfg.pins.io3_pin = 21;

	driver_data->qspi_cfg.prot_if.readoc = 4;
	driver_data->qspi_cfg.prot_if.writeoc = 3;

	driver_data->qspi_cfg.phy_if.sck_freq = 1;

	ret = nrfx_qspi_init(&driver_data->qspi_cfg, qspi_handler, (void *)dev);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("QSPI init error: 0x%x", ret);
		return ret;
	}

    ret = 0;

	nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = QSPI_STD_CMD_RSTEN,
		.length = NRF_QSPI_CINSTR_LEN_1B,
		.io2_level = true,
		.io3_level = true,
		.wipwait = true,
		.wren = true
    };

	/* Send reset enable */
	ret = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("QSPI reset enable command error: 0x%x", ret);
        return -EIO;
	}

	/* Send reset command */
	cinstr_cfg.opcode = QSPI_STD_CMD_RST;
	ret = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("QSPI reset command error: 0x%x", ret);
        return -EIO;
	}

	/* Get 3 byte identification value */
	uint8_t rdid_buf[3] = { 0, 0, 0 };
	cinstr_cfg.opcode = QSPI_STD_CMD_READ_ID;
	cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_4B;
	ret = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, rdid_buf);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("QSPI get 3 byte id error: 0x%x", ret);
        return -EIO;
	}

	if (rdid_buf[0] == 0xC2 && rdid_buf[1] == 0x28 && rdid_buf[2] == 0x17) {
		LOG_INF("MX25RXX flash detected!");
	}
    else
    {
        LOG_ERR("flash not found, got id: 0x%x 0x%x 0x%x",
                rdid_buf[0], rdid_buf[1], rdid_buf[2]);
    }

    /* Switch to qspi mode */
	cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
	cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
	ret = nrfx_qspi_cinstr_xfer(&cinstr_cfg, &temporary, NULL);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("QSPI set mode to x4: 0x%x", ret);
        return -EIO;
	}

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

/* instance 0 size in bytes */
#define INST_0_BYTES QSPI_FLASH_CHIP_SIZE

/* instance 0 page count */
#define LAYOUT_PAGES_COUNT (INST_0_BYTES / QSPI_ERASE_SECTOR_SIZE)//QSPI_FLASH_PAGE_PROGRAM_SIZE)

BUILD_ASSERT_MSG((QSPI_ERASE_SECTOR_SIZE * LAYOUT_PAGES_COUNT)
		 == INST_0_BYTES,
		 "QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE incompatible with flash size");

static const struct flash_pages_layout dev_layout = {
	.pages_count = LAYOUT_PAGES_COUNT,
	.pages_size = QSPI_ERASE_SECTOR_SIZE,//QSPI_FLASH_PAGE_PROGRAM_SIZE,
};
#undef LAYOUT_PAGES_COUNT

static void block_dev_pages_layout(struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api qspi_flash_api = {
	.read = block_dev_qspi_read_req,
	.write = block_dev_qspi_write_req,
	.erase = block_dev_qspi_erase,
	.write_protection = block_dev_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = block_dev_pages_layout,
#endif
	.write_block_size = 4,
};

static struct qspi_flash_data qspi_flash_memory_data;

DEVICE_AND_API_INIT(qspi_flash_memory, DT_INST_0_QSPI_MX25RXX_LABEL,
		    block_dev_qspi_init, &qspi_flash_memory_data, NULL,
		    POST_KERNEL, CONFIG_QSPI_FLASH_MX25RXX_INIT_PRIORITY,
		    &qspi_flash_api);

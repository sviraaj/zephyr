#ifndef  ZEPHYR_DRIVERS_FLASH_QSPI_FLASH_MX25RXX_H_
#define  ZEPHYR_DRIVERS_FLASH_QSPI_FLASH_MX25RXX_H_

#include <nrfx_qspi.h>

#ifdef __cplusplus
extern "C" {
#endif

struct qspi_flash_data {
	//struct device *qspi;
    nrfx_qspi_config_t qspi_cfg;
#if defined(CONFIG_MULTITHREADING)
	struct k_sem sem;
#endif /* CONFIG_MULTITHREADING */
	struct k_sem sem_op;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_FLASH_QSPI_FLASH_MX25RXX_H_ */

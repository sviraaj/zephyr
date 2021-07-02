#
#  Copyright (c) 2018 Analog Devices Inc.
#
#  SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ADXL357 adxl357.c)
zephyr_library_sources_ifdef(CONFIG_ADXL357_TRIGGER adxl357_trigger.c)

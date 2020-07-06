# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(zbacrux_new_app)

target_include_directories(app PRIVATE include)
target_include_directories(app PRIVATE src)
target_include_directories(app PRIVATE src/utils/cJSON)
#target_include_directories(app PRIVATE src/display3/FT_Esd_Framework)
#target_include_directories(app PRIVATE src/display3/FT_Esd_Widgets)
#target_include_directories(app PRIVATE src/display3/FT_Eve_Hal)
#target_include_directories(app PRIVATE src/display3/FT_Eve_Hal/Hdr)
#target_include_directories(app PRIVATE src/display3/MenuPage)

FILE(GLOB app_src_top src/*.c)
FILE(GLOB utils_cjson src/utils/cJSON/cJSON.c)

target_sources(app PRIVATE ${app_src_top})
target_sources(app PRIVATE ${utils_cjson})

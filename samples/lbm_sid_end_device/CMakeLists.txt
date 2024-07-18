#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

# Sidewalk version
include(bootloader_version.cmake)

# Child images
set(hci_ipc_KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/child_image/hci_ipc/Kconfig.root)
set(mcuboot_KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/child_image/mcuboot/Kconfig.root)

# Configurations
if(CONF_FILE)
    get_filename_component(CONFIG_FILE_NAME ${CONF_FILE} NAME)
endif()

if("${CONFIG_FILE_NAME}" STREQUAL "prj_no_dfu.conf")
    set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/configuration/pm_static_no_dfu.yml)
else()
    set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/configuration/${BOARD}/pm_static_dfu.yml)
endif()

# Zephyr CMake project
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sidewalk_sid_end_device)

# Source files
target_include_directories(app PRIVATE include)
target_sources(app PRIVATE
    src/main.c
    src/sidewalk.c
)

target_sources_ifdef(CONFIG_SIDEWALK_FILE_TRANSFER app PRIVATE src/file_transfer.c)

if(CONFIG_SID_END_DEVICE_SENSOR_MONITORING)
    target_sources(app PRIVATE
        src/sensor_monitoring/app_buttons.c
        src/sensor_monitoring/app_leds.c
        src/sensor_monitoring/app_sensor.c
        src/sensor_monitoring/app_tx.c
        src/sensor_monitoring/app_rx.c
        src/sensor_monitoring/app.c
    )
elseif(CONFIG_SID_END_DEVICE_DUT)
    target_sources(app PRIVATE
        src/cli/app.c
    )
elseif(CONFIG_LBM_END_DEVICE)
    target_sources(app PRIVATE
        src/lbm/app_lbm_sidewalk.c
        src/lbm/main_periodical_uplink.c
    )
	zephyr_compile_definitions(
		CRYPTO=SOFT
		RP2_103
		SMTC_D2D
		USER_DEFINED_JOIN_PARAMETERS
	)
	if (CONFIG_REGION_US_915)
		target_compile_definitions(app PRIVATE MODEM_EXAMPLE_REGION=SMTC_MODEM_REGION_US_915)
	else()
		message(SEND_ERROR "not us_915 region")
	endif()
elseif(CONFIG_SID_END_DEVICE_NAV3)
	if(CONFIG_NAV3_SEND)
		target_compile_definitions(app PRIVATE NAV3_SEND)
		target_sources(app PRIVATE src/lbm/wifi.c)
	elseif(CONFIG_NAV3_ONLY)
		target_compile_definitions(app PRIVATE NAV3_ONLY)
	endif()
    target_sources(app PRIVATE
        src/lbm/app_nav3_sidewalk.c
        src/lbm/main_geolocation_sidewalk.c
    )
	zephyr_compile_definitions(RP2_103)	# TODO why should this be necessary in bypass
	if (CONFIG_REGION_US_915)
		# TODO why should this be necessary in bypass
		target_compile_definitions(app PRIVATE MODEM_EXAMPLE_REGION=SMTC_MODEM_REGION_US_915)
	else()
		message(SEND_ERROR "not us_915 region")
	endif()
elseif(CONFIG_LBM_END_DEVICE_NAV3)
    target_sources(app PRIVATE
        src/lbm/app_nav3_lbm.c
        src/lbm/main_geolocation_lbm.c
    )
	zephyr_compile_definitions(RP2_103)
	if (CONFIG_REGION_US_915)
		target_compile_definitions(app PRIVATE MODEM_EXAMPLE_REGION=SMTC_MODEM_REGION_US_915)
	else()
		message(SEND_ERROR "not us_915 region")
	endif()
else()
    target_sources(app PRIVATE
        src/hello/app.c
    )
endif()

if(CONFIG_SID_END_DEVICE_CLI)
    target_sources(app PRIVATE
        src/cli/app_dut.c
        src/cli/app_shell.c
    )
endif()

if(CONFIG_SMTC_CLI)
    target_sources(app PRIVATE
		src/cli/smtc_shell.c
	)
	target_compile_definitions(app PRIVATE CONFIG_SMTC_CLI)
endif()

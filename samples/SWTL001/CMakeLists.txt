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
)

if(CONFIG_SID_END_DEVICE_SWTL001)
    target_sources(app PRIVATE
        src/swtl001/app.c
		src/swtl001/lr11xx_firmware_update.c
    )
else()
	message(SEND_ERROR "only swtl001")
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

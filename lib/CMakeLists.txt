# SPDX-License-Identifier: Apache-2.0

if(CONFIG_RADIO_TIMING_TEST)
	zephyr_library_sources(radio_timing_test.c)
	zephyr_compile_definitions(TIMING_TEST)
	zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(CONFIG_RADIO_LR11XX)
	if(CONFIG_LR1110_CLI)
		zephyr_library_sources(lr1110_shell.c)
	endif()

	add_subdirectory(sid_pal_lr11xx)

    add_library(lr11xx_driver INTERFACE)
    target_link_libraries(lr11xx_driver INTERFACE zephyr_interface)
    zephyr_library()
    zephyr_library_sources(
        lr11xx_driver/src/lr11xx_system.c
        lr11xx_driver/src/lr11xx_lr_fhss.c
        lr11xx_driver/src/lr11xx_radio.c
        lr11xx_driver/src/lr11xx_regmem.c
        lr11xx_driver/src/lr11xx_bootloader.c
        lr11xx_driver/src/lr11xx_gnss.c
        lr11xx_driver/src/lr11xx_wifi.c
        app_subGHz_config_lr11xx.c
    )
    zephyr_include_directories(
        lr11xx_driver/src
    )

	zephyr_library_link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/${RADIO_LIB_NAME})

elseif(CONFIG_RADIO_SX126X)
	add_subdirectory(sid_pal_sx126x)
endif()

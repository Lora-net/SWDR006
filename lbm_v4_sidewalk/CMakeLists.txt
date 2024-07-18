
# global includes
zephyr_include_directories(
	RALBSP
	.
)

if(CONFIG_RADIO_LR11XX)
	set(RADIO_SRC
		RALBSP/ral_lr11xx_bsp.c
	)
else()
	set(RADIO_SRC
		RadioDriverHAL/sx126x.c
		RALBSP/ral_sx126x_bsp.c
	)
endif()

set(SRC
	radio_utilities.c
	ModemHAL/smtc_modem_hal.c
	${RADIO_SRC}
)

zephyr_interface_library_named(lbm_sidewalk)

target_include_directories(lbm_sidewalk INTERFACE
	../${LBM_DIR}/smtc_modem_core
	RadioDriverHAL
)

if(CONFIG_RADIO_LR11XX)
else()
    target_include_directories(lbm_sidewalk INTERFACE
		../${LBM_DIR}/smtc_modem_core/radio_drivers/sx126x_driver/src
	    sx126x/include
	    sx126x/include/semtech  # sx126x_radio.h wants sx126x_halo.h
    )
endif()

if(DEFINED PORTING_TEST) # LBM lib not built
	target_include_directories(lbm_sidewalk INTERFACE
		../${LBM_DIR}/smtc_modem_core/smtc_ral/src
		../${LBM_DIR}/smtc_modem_core/smtc_ralf/src
		../${LBM_DIR}/smtc_modem_hal
		../${LBM_DIR}/smtc_modem_api
	)
	target_compile_definitions(lbm_sidewalk INTERFACE PORTING_TEST)
endif()

if(CONFIG_RADIO_LR11XX)
    target_compile_definitions(lbm_sidewalk INTERFACE LR11XX)
else()
    message(SEND_ERROR "board definition for other radio SX128X, SX126X, SX127X")
endif()

if(CONFIG_SMTC_MODEM_HAL_IRQ_FROM_SID_PAL)
    target_compile_definitions(lbm_sidewalk INTERFACE SMTC_MODEM_HAL_IRQ_FROM_SID_PAL)
endif()

if(CONFIG_LBM_REBOOT_ON_PANIC)
    target_compile_definitions(lbm_sidewalk INTERFACE LBM_REBOOT_ON_PANIC)
endif()

zephyr_library()
zephyr_library_sources(${SRC})
zephyr_library_link_libraries(lbm_sidewalk)

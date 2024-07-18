
# global include, application needs halo_lr11xx_radio.h
zephyr_include_directories(
	include
)

set(LR11XX_SOURCE
	lr11xx_radio.c
	lr11xx_hal.c
	lr11xx_radio_lora.c
	lr11xx_radio_fsk.c
	semtech/lr11xx_halo.c
)

add_library(SID_LR11XX INTERFACE)

if(NOT DEFINED DUAL_LINK_SUPPORT)
	if(CONFIG_SIDEWALK_LINK_MASK_FSK)
		# for "sid init 3" (lora only) in sid_dut, then DUAL_LINK_SUPPORT must be 1
		target_compile_definitions(SID_LR11XX INTERFACE DUAL_LINK_SUPPORT=0)
	else()
		target_compile_definitions(SID_LR11XX INTERFACE DUAL_LINK_SUPPORT=1)
	endif()
endif()


target_link_libraries(SID_LR11XX INTERFACE zephyr_interface)
target_include_directories(SID_LR11XX INTERFACE
	include/semtech
	..
)

if(CONFIG_SMTC_MODEM_HAL_IRQ_FROM_SID_PAL)
	target_compile_definitions(SID_LR11XX INTERFACE SMTC_MODEM_HAL_IRQ_FROM_SID_PAL)
endif()

target_compile_definitions(SID_LR11XX INTERFACE
	FSK_SUPPRESS_RX_TIMEOUT
)
#	LORA_SUPPRESS_RX_TIMEOUT

zephyr_library()
zephyr_library_sources(${LR11XX_SOURCE})
zephyr_library_link_libraries(SID_LR11XX)


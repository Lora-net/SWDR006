
# global includes
zephyr_include_directories(
	smtc_ral/src
	smtc_ralf/src
    geolocation_services
	.
)

zephyr_interface_library_named(smtc_lbm)

if(DEFINED PORTING_TEST)
	set(LBM_SOURCE
		${RADIO_SRC}
	)
else()
	set(LBM_SOURCE
		smtc_modem.c
		modem_utilities/modem_core.c
		modem_utilities/modem_event_utilities.c
		modem_utilities/fifo_ctrl.c
		modem_supervisor/modem_supervisor_light.c
		lorawan_api/lorawan_api.c
		radio_planner/src/radio_planner.c
		smtc_modem_crypto/soft_secure_element/soft_se.c # or smtc_modem_crypto/lr11xx_crypto_engine/lr11xx_ce.c
		lr1mac/src/smtc_real/src/smtc_real.c
		lr1mac/src/services/smtc_duty_cycle.c
		lr1mac/src/lr1mac_core.c
		lr1mac/src/lr1_stack_mac_layer.c
		lr1mac/src/lr1mac_utilities.c
		lr1mac/src/services/smtc_lbt.c
		lorawan_manager/lorawan_send_management.c
		lorawan_manager/lorawan_join_management.c
		lorawan_manager/lorawan_dwn_ack_management.c
		lorawan_manager/lorawan_cid_request_management.c
		lorawan_packages/lorawan_certification/lorawan_certification.c
		smtc_modem_crypto/smtc_modem_crypto.c
		smtc_modem_crypto/soft_secure_element/cmac.c
		smtc_modem_crypto/soft_secure_element/aes.c
		smtc_modem_test.c
		modem_services/cloud_dm_package/cloud_dm_package.c
		modem_services/lfu_service/file_upload.c
		modem_services/stream_packages/stream.c
		modem_services/stream_packages/rose.c
		lr1mac/src/services/smtc_lora_cad_bt.c
		lorawan_packages/application_layer_clock_synchronization/v2.0.0/lorawan_alcsync_v2.0.0.c
	)

    if(CONFIG_ADD_LBM_GEOLOCATION)
	    target_compile_definitions(smtc_lbm INTERFACE ADD_LBM_GEOLOCATION)
	    set(LBM_SOURCE ${LBM_SOURCE}
            geolocation_services/mw_gnss_almanac.c
            geolocation_services/mw_gnss_scan.c
            geolocation_services/mw_wifi_scan.c
            geolocation_services/mw_wifi_send.c
            geolocation_services/mw_common.c
            geolocation_services/wifi_helpers.c
            geolocation_services/gnss_helpers.c
            geolocation_services/mw_gnss_send.c
        )
    endif()
endif()

if(CONFIG_RADIO_LR11XX)
	set(LBM_SOURCE ${LBM_SOURCE}
		smtc_ralf/src/ralf_lr11xx.c
		smtc_ral/src/ral_lr11xx.c
	)
else()
	set(LBM_SOURCE ${LBM_SOURCE}
		smtc_ral/src/ral_sx126x.c
		smtc_ralf/src/ralf_sx126x.c
		radio_drivers/sx126x_driver/src/sx126x_lr_fhss.c
		radio_drivers/sx126x_driver/src/lr_fhss_mac.c
	)
endif()

target_compile_definitions(smtc_lbm INTERFACE
	NUMBER_OF_STACKS=1 # NB_OF_STACK
)

if (CONFIG_TEST_BYPASS_JOIN_DUTY_CYCLE)
	target_compile_definitions(smtc_lbm INTERFACE TEST_BYPASS_JOIN_DUTY_CYCLE)
endif()

if (CONFIG_LBM_HAL_DBG)
   target_compile_definitions(smtc_lbm INTERFACE MODEM_HAL_DBG_TRACE=1)
else()
   target_compile_definitions(smtc_lbm INTERFACE MODEM_HAL_DBG_TRACE=0)
endif()

if (CONFIG_LBM_HAL_DBG_DEEP)
   target_compile_definitions(smtc_lbm INTERFACE MODEM_HAL_DEEP_DBG_TRACE=1)
else()
   target_compile_definitions(smtc_lbm INTERFACE MODEM_HAL_DEEP_DBG_TRACE=0)
endif()

if(CONFIG_LBM_GNSS_ALMANAC_DEEP_DBG)
   target_compile_definitions(smtc_lbm INTERFACE GNSS_ALMANAC_DEEP_DBG_TRACE)
endif()

if(CONFIG_LBM_GNSS_SCAN_DEEP_DBG)
   target_compile_definitions(smtc_lbm INTERFACE GNSS_SCAN_DEEP_DBG_TRACE)
endif()


if(CONFIG_RADIO_LR11XX)
    target_compile_definitions(smtc_lbm INTERFACE LR11XX)
else()
    message(SEND_ERROR "board definition for other radio SX128X, SX126X, SX127X")
endif()


# includes private to LBM
if(DEFINED PORTING_TEST)
	target_compile_definitions(smtc_lbm INTERFACE PORTING_TEST)
	target_include_directories(smtc_lbm INTERFACE
		radio_drivers/sx126x_driver/src
	)
else()
	if (CONFIG_REGION_US_915)
		target_compile_definitions(smtc_lbm INTERFACE REGION_US_915)
		set(LBM_SOURCE ${LBM_SOURCE} lr1mac/src/smtc_real/src/region_us_915.c)
	elseif (CONFIG_REGION_EU868)
		target_compile_definitions(smtc_lbm INTERFACE REGION_EU_868)
		set(LBM_SOURCE ${LBM_SOURCE} lr1mac/src/smtc_real/src/region_eu_868.c)
	else()
		message(FATAL_ERROR "other region defined")
	endif()

	target_include_directories(smtc_lbm INTERFACE
		lorawan_manager
		lr1mac/src
		smtc_modem_crypto/smtc_secure_element
		lr1mac/src/smtc_real/src
		radio_planner/src
		lr1mac/src/services
		modem_supervisor
		modem_utilities
		lorawan_packages/lorawan_certification
		lorawan_api
		smtc_modem_crypto
		..
		modem_config
		lr1mac/src/services/smtc_multicast
		lr1mac
		radio_drivers/sx126x_driver/src
		modem_services/cloud_dm_package
		modem_services
		modem_services/lfu_service
		modem_services/stream_packages
		lorawan_packages/application_layer_clock_synchronization
	)
endif()

zephyr_library()
zephyr_library_sources(${LBM_SOURCE})
zephyr_library_link_libraries(smtc_lbm)


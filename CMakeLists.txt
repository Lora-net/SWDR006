if(CONFIG_LORABASICSMODEM_V450)
	add_subdirectory(smtc_modem_core)

	# global includes
	zephyr_include_directories(
    	smtc_modem_api
    	smtc_modem_hal
	)
endif()


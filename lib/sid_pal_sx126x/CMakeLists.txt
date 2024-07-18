
set(SX126X_SOURCE
	sx126x_radio.c
	sx126x_radio_lora.c
	sx126x_radio_fsk.c
	semtech/sx126x.c
	sx126x_hal.c
	semtech/sx126x_halo.c
)

add_library(SID_SX126X INTERFACE)
target_link_libraries(SID_SX126X INTERFACE zephyr_interface)

target_include_directories(SID_SX126X INTERFACE
	include
	include/semtech
	..
)

zephyr_library()
zephyr_library_sources(${SX126X_SOURCE})
zephyr_library_link_libraries(SID_SX126X)

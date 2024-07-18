DEST_DIR=$1
if [ ! -d $DEST_DIR ]; then
    echo "dest dir $DEST_DIR not found!"
	exit 1
fi

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-demo.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_LORA=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_demo_lr11xx_xtal_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-demo.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_LORA=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_demo_lr11xx_tcxo_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-demo.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_FSK=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_demo_lr11xx_xtal_fsk.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-demo.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_FSK=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_demo_lr11xx_tcxo_fsk.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-demo.conf -DCONFIG_RADIO_LR11XX=n -DCONFIG_RADIO_SX126X=y -DCONFIG_SIDEWALK_LINK_MASK_FSK=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_demo_sx126x_fsk.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-demo.conf -DCONFIG_RADIO_LR11XX=n -DCONFIG_RADIO_SX126X=y -DCONFIG_SIDEWALK_LINK_MASK_LORA=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_demo_sx126x_lora.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-dut.conf -DCONFIG_RADIO_LR11XX=n -DCONFIG_RADIO_SX126X=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_dut_sx126x.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-dut.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_SX126X=n -DCONFIG_RADIO_TCXO=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_dut_lr11xx_tcxo.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-dut.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_SX126X=n -DCONFIG_RADIO_TCXO=n
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_dut_lr11xx_xtal.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-hello.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_LORA=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_hello_lr11xx_xtal_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-hello.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_LORA=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_hello_lr11xx_tcxo_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-hello.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_FSK=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_hello_lr11xx_xtal_fsk.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-hello.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_FSK=y
cp build/zephyr/merged.hex $DEST_DIR/sid_end_device_hello_lr11xx_tcxo_fsk.hex

DEST_DIR=$1
if [ ! -d $DEST_DIR ]; then
    echo "dest dir $DEST_DIR not found!"
	exit 1
fi

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm431_lr11xx_xtal_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm431_lr11xx_tcxo_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_FSK=y -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm431_lr11xx_xtal_fsk.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_FSK=y -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm431_lr11xx_tcxo_fsk.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm450_lr11xx_xtal_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm450_lr11xx_tcxo_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_FSK=y -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm450_lr11xx_xtal_fsk.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_FSK=y -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_lbm450_lr11xx_tcxo_fsk.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_nav3lbm431_lr11xx_xtal.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_nav3lbm431_lr11xx_tcxo.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_nav3lbm450_lr11xx_xtal.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3lbm.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_nav3lbm450_lr11xx_tcxo.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3sid.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_v431_nav3sid_lr11xx_xtal_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3sid.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V431=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_v431_nav3sid_lr11xx_tcxo_lora.hex

west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3sid.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=n -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_v450_nav3sid_lr11xx_xtal_lora.hex
west build -b nrf52840dk_nrf52840 -- -DOVERLAY_CONFIG=overlay-nav3sid.conf -DCONFIG_RADIO_LR11XX=y -DCONFIG_RADIO_TCXO=y -DCONFIG_SIDEWALK_LINK_MASK_LORA=y -DCONFIG_LORABASICSMODEM_V450=y
cp build/zephyr/merged.hex $DEST_DIR/lbm_sid_end_device_v450_nav3sid_lr11xx_tcxo_lora.hex


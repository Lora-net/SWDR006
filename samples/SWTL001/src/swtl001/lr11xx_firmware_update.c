#include <lr11xx_firmware_update.h>
#include <lr11xx_bootloader.h>
#include <app_subGHz_config.h>
#include <sid_pal_gpio_ifc.h>
#include <halo_lr11xx_radio.h>
#include <sid_pal_delay_ifc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fwup, CONFIG_SIDEWALK_LOG_LEVEL);

#define LR11XX_TYPE_PRODUCTION_MODE 0xDF

bool lr11xx_is_chip_in_production_mode( uint8_t type )
{
	return ( type == LR11XX_TYPE_PRODUCTION_MODE ) ? true : false;
}

bool lr11xx_is_fw_compatible_with_chip( lr11xx_fw_update_t update, uint16_t bootloader_version )
{
	if( ( ( update == LR1110_FIRMWARE_UPDATE_TO_TRX ) || ( update == LR1110_FIRMWARE_UPDATE_TO_MODEM ) ) &&
		( bootloader_version != 0x6500 ) )
	{
		return false;
	}
	else if( ( update == LR1120_FIRMWARE_UPDATE_TO_TRX ) && ( bootloader_version != 0x2000 ) )
	{
		return false;
	}
	else if( ( update == LR1121_FIRMWARE_UPDATE_TO_TRX ) && ( bootloader_version != 0x2100 ) )
	{
		return false;
	}

	return true;
}

lr11xx_fw_update_status_t lr11xx_update_firmware(lr11xx_fw_update_t fw_update_direction,
	uint32_t fw_expected,
	const uint32_t* buffer,
	uint32_t length)
{
	lr11xx_bootloader_version_t version_bootloader = { 0 };
	void *drv_ctx;
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	
	LOG_INF( "Reset the chip..." );
	if (sid_pal_gpio_set_direction(cfg->gpios.radio_busy,
		SID_PAL_GPIO_DIRECTION_OUTPUT) != SID_ERROR_NONE) {
		LOG_ERR("set busy dir out");
		return LR11XX_FW_UPDATE_ERROR;
	}
	sid_pal_gpio_write(cfg->gpios.radio_busy, 0);

	drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_reset(drv_ctx) != LR11XX_STATUS_OK) {
		LOG_ERR("lr11xx_system_reset fail");
		return LR11XX_FW_UPDATE_ERROR;
	}
	sid_pal_delay_us(500000);

	if (sid_pal_gpio_set_direction(cfg->gpios.radio_busy,
		SID_PAL_GPIO_DIRECTION_INPUT) != SID_ERROR_NONE) {
		LOG_ERR("set busy dir in");
		return LR11XX_FW_UPDATE_ERROR;
	}
	sid_pal_delay_us(100000);

	LOG_INF( "> Reset done!" );	

	lr11xx_bootloader_get_version(drv_ctx, &version_bootloader);
	LOG_INF( "Chip in bootloader mode:" );
	LOG_INF( " - Chip type			   = 0x%02X (0xDF for production)", version_bootloader.type );
	LOG_INF( " - Chip hardware version   = 0x%02X (0x22 for V2C)", version_bootloader.hw );
	LOG_INF( " - Chip bootloader version = 0x%04X", version_bootloader.fw );

	if( lr11xx_is_chip_in_production_mode( version_bootloader.type ) == false )
	{
		LOG_ERR("not in production mode");
		return LR11XX_FW_UPDATE_WRONG_CHIP_TYPE;
	}

	if( lr11xx_is_fw_compatible_with_chip( fw_update_direction, version_bootloader.fw ) == false )
	{
		LOG_ERR("incompatible firmware");
		return LR11XX_FW_UPDATE_WRONG_CHIP_TYPE;
	};

	LOG_INF( "Start flash erase..." );
	lr11xx_bootloader_erase_flash( drv_ctx );
	LOG_INF( "> Flash erase done!" );

	LOG_INF( "Start flashing firmware... %u", length);
	lr11xx_bootloader_write_flash_encrypted_full( drv_ctx, 0, buffer, length );
	LOG_INF( "> Flashing done!" );

	LOG_INF( "Rebooting..." );
	lr11xx_bootloader_reboot( drv_ctx, false );
	LOG_INF( "> Reboot done!" );

	switch( fw_update_direction )
	{
	case LR1110_FIRMWARE_UPDATE_TO_TRX:
	case LR1120_FIRMWARE_UPDATE_TO_TRX:
	case LR1121_FIRMWARE_UPDATE_TO_TRX:
	{
		lr11xx_system_version_t version_trx = { 0x00 };
		lr11xx_system_uid_t	 uid		 = { 0x00 };

		lr11xx_system_get_version( drv_ctx, &version_trx );
		LOG_INF( "Chip in transceiver mode:" );
		LOG_INF( " - Chip type			 = 0x%02X", version_trx.type );
		LOG_INF( " - Chip hardware version = 0x%02X", version_trx.hw );
		LOG_INF( " - Chip firmware version = 0x%04X", version_trx.fw );

		lr11xx_system_read_uid( drv_ctx, uid );

		if( version_trx.fw == fw_expected )
		{
			return LR11XX_FW_UPDATE_OK;
		}
		else
		{
			return LR11XX_FW_UPDATE_ERROR;
		}
		break;
	}
	case LR1110_FIRMWARE_UPDATE_TO_MODEM:
	{
		/* not using radio in lorawan mode for sidewalk */
		LOG_ERR("LR1110_FIRMWARE_UPDATE_TO_MODEM");
	}
	} // ..switch( fw_update_direction )

	return LR11XX_FW_UPDATE_ERROR;
}


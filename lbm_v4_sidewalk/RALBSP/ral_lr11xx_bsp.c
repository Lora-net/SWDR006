#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <string.h>

#include <lr11xx_config.h>
#include <app_subGHz_config.h>

#include "ral_lr11xx_bsp.h"
#include "ralf_lr11xx.h"

#include <halo_lr11xx_radio.h>
#include "smtc_modem_utilities.h"

#include "sid_pal_gpio_ifc.h"

#include <zephyr/logging/log.h>
#ifdef PORTING_TEST
LOG_MODULE_REGISTER(RALBSP, CONFIG_SIDEWALK_LOG_LEVEL);
#else
LOG_MODULE_REGISTER(RALBSPModemHAL, CONFIG_LBM_LOG_LEVEL);
#endif

const radio_lr11xx_device_config_t *lr_cfg;

ralf_t *smtc_board_initialise_and_get_ralf(void)
{
	static ralf_t localRalf = {0};
	const void *halo_ctx = lr11xx_get_drv_ctx();
	localRalf = (ralf_t) RALF_LR11XX_INSTANTIATE(halo_ctx);
	LOG_INF("smtc_board_initialise_and_get_ralf %p, halo:%p ", &localRalf.ral, halo_ctx);
	smtc_modem_set_radio_context(halo_ctx);

   	lr_cfg = get_radio_cfg();

	return &localRalf;
}

void ral_lr11xx_bsp_get_rssi_calibration_table( const void* context, const uint32_t freq_in_hz,
			lr11xx_radio_rssi_calibration_table_t* rssi_calibration_table )
{
	if( freq_in_hz <= 600000000 )
	{
		rssi_calibration_table->gain_offset		= 0;
		rssi_calibration_table->gain_tune.g4	= 12;
		rssi_calibration_table->gain_tune.g5	= 12;
		rssi_calibration_table->gain_tune.g6	= 14;
		rssi_calibration_table->gain_tune.g7	= 0;
		rssi_calibration_table->gain_tune.g8	= 1;
		rssi_calibration_table->gain_tune.g9	= 3;
		rssi_calibration_table->gain_tune.g10	= 4;
		rssi_calibration_table->gain_tune.g11	= 4;
		rssi_calibration_table->gain_tune.g12	= 3;
		rssi_calibration_table->gain_tune.g13	= 6;
		rssi_calibration_table->gain_tune.g13hp1 = 6;
		rssi_calibration_table->gain_tune.g13hp2 = 6;
		rssi_calibration_table->gain_tune.g13hp3 = 6;
		rssi_calibration_table->gain_tune.g13hp4 = 6;
		rssi_calibration_table->gain_tune.g13hp5 = 6;
		rssi_calibration_table->gain_tune.g13hp6 = 6;
		rssi_calibration_table->gain_tune.g13hp7 = 6;
	}
	else if( ( 600000000 <= freq_in_hz ) && ( freq_in_hz <= 2000000000 ) )
	{
		rssi_calibration_table->gain_offset		= 0;
		rssi_calibration_table->gain_tune.g4	= 2;
		rssi_calibration_table->gain_tune.g5	= 2;
		rssi_calibration_table->gain_tune.g6	= 2;
		rssi_calibration_table->gain_tune.g7	= 3;
		rssi_calibration_table->gain_tune.g8	= 3;
		rssi_calibration_table->gain_tune.g9	= 4;
		rssi_calibration_table->gain_tune.g10	= 5;
		rssi_calibration_table->gain_tune.g11	= 4;
		rssi_calibration_table->gain_tune.g12	= 4;
		rssi_calibration_table->gain_tune.g13	= 6;
		rssi_calibration_table->gain_tune.g13hp1 = 5;
		rssi_calibration_table->gain_tune.g13hp2 = 5;
		rssi_calibration_table->gain_tune.g13hp3 = 6;
		rssi_calibration_table->gain_tune.g13hp4 = 6;
		rssi_calibration_table->gain_tune.g13hp5 = 6;
		rssi_calibration_table->gain_tune.g13hp6 = 7;
		rssi_calibration_table->gain_tune.g13hp7 = 6;
	}
	else  // freq_in_hz > 2000000000
	{
		rssi_calibration_table->gain_offset		= 2030;
		rssi_calibration_table->gain_tune.g4	= 6;
		rssi_calibration_table->gain_tune.g5	= 7;
		rssi_calibration_table->gain_tune.g6	= 6;
		rssi_calibration_table->gain_tune.g7	= 4;
		rssi_calibration_table->gain_tune.g8	= 3;
		rssi_calibration_table->gain_tune.g9	= 4;
		rssi_calibration_table->gain_tune.g10	= 14;
		rssi_calibration_table->gain_tune.g11	= 12;
		rssi_calibration_table->gain_tune.g12	= 14;
		rssi_calibration_table->gain_tune.g13	= 12;
		rssi_calibration_table->gain_tune.g13hp1 = 12;
		rssi_calibration_table->gain_tune.g13hp2 = 12;
		rssi_calibration_table->gain_tune.g13hp3 = 12;
		rssi_calibration_table->gain_tune.g13hp4 = 8;
		rssi_calibration_table->gain_tune.g13hp5 = 8;
		rssi_calibration_table->gain_tune.g13hp6 = 9;
		rssi_calibration_table->gain_tune.g13hp7 = 9;
	}
}

void ral_lr11xx_bsp_get_lora_cad_det_peak( ral_lora_sf_t sf, ral_lora_bw_t bw, ral_lora_cad_symbs_t nb_symbol,
				uint8_t* in_out_cad_det_peak )
{
	// Function used to fine tune the cad detection peak, update if needed
}

void ral_lr11xx_bsp_get_tx_cfg( const void* context, const ral_lr11xx_bsp_tx_cfg_input_params_t* input_params,
				ral_lr11xx_bsp_tx_cfg_output_params_t* output_params )
{
	int16_t power = input_params->system_output_pwr_in_dbm;// + board_tx_pwr_offset_db;
	radio_lr11xx_pa_cfg_t pa_cfg;

	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	cfg->pa_cfg_callback(power, &pa_cfg);

	output_params->pa_ramp_time = pa_cfg.ramp_time;
	output_params->chip_output_pwr_in_dbm_expected = power;

	output_params->pa_cfg.pa_sel		= pa_cfg.pa_cfg.pa_sel;
	output_params->pa_cfg.pa_reg_supply = pa_cfg.pa_cfg.pa_reg_supply;
	output_params->pa_cfg.pa_duty_cycle = pa_cfg.pa_cfg.pa_duty_cycle;
	output_params->pa_cfg.pa_hp_sel		= pa_cfg.pa_cfg.pa_hp_sel;
	output_params->chip_output_pwr_in_dbm_configured = pa_cfg.tx_power_in_dbm;
}


void ral_lr11xx_bsp_get_lfclk_cfg_in_sleep( const void* context, bool* lfclk_is_running )
{
	*lfclk_is_running = true;
}

void ral_lr11xx_bsp_get_rx_boost_cfg( const void* context, bool* rx_boost_is_activated )
{
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	*rx_boost_is_activated = cfg->rx_boost;
}

void ral_lr11xx_bsp_get_xosc_cfg( const void* context, ral_xosc_cfg_t* xosc_cfg,
			lr11xx_system_tcxo_supply_voltage_t* supply_voltage, uint32_t* startup_time_in_tick )
{
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	/* RAL_XOSC_CFG_XTAL
	 * RAL_XOSC_CFG_TCXO_RADIO_CTRL
	 * RAL_XOSC_CFG_TCXO_EXT_CTRL */
	switch (cfg->tcxo_config.ctrl) {
		case LR11XX_TCXO_CTRL_NONE:
			*xosc_cfg = RAL_XOSC_CFG_XTAL;
			break;
		case LR11XX_TCXO_CTRL_VDD:
			*xosc_cfg = RAL_XOSC_CFG_TCXO_EXT_CTRL;	 /* TODO is this correct */
			break;
		case LR11XX_TCXO_CTRL_DIO3:
			*xosc_cfg =RAL_XOSC_CFG_TCXO_RADIO_CTRL;
			break;
	}
	*supply_voltage			= cfg->tcxo_config.tune;
	// tick is 30.52µs
	*startup_time_in_tick 	= cfg->tcxo_config.timeout;
}

void ral_lr11xx_bsp_get_rf_switch_cfg( const void* context, lr11xx_system_rfswitch_cfg_t* rf_switch_cfg )
{
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	memcpy(rf_switch_cfg, &cfg->rfswitch, sizeof(lr11xx_system_rfswitch_cfg_t));
}

void ral_lr11xx_bsp_get_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode )
{
	/* this is reduendant with radio_lr11xx_device_config_t.regulator_mode in app_subGHz_config_lr11xx.c */
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	*reg_mode = cfg->regulator_mode;
}

void ral_lr11xx_bsp_get_crc_state( const void* context, bool* crc_is_activated )
{
	*crc_is_activated = false;
}

void geolocation_bsp_gnss_get_consumption( lr11xx_gnss_instantaneous_power_consumption_ua_t* instantaneous_power_consumption_ua )
{
	/* These value are for EVK board in DC DC mode with Xtal 32KHz and a TCXO 32MHz*/
	instantaneous_power_consumption_ua->board_voltage_mv			  = 3300;
	instantaneous_power_consumption_ua->init_ua					   = 3150;
	instantaneous_power_consumption_ua->phase1_gps_capture_ua		 = 11900;
	instantaneous_power_consumption_ua->phase1_gps_process_ua		 = 3340;
	instantaneous_power_consumption_ua->multiscan_gps_capture_ua	  = 10700;
	instantaneous_power_consumption_ua->multiscan_gps_process_ua	  = 4180;
	instantaneous_power_consumption_ua->phase1_beidou_capture_ua	  = 13500;
	instantaneous_power_consumption_ua->phase1_beidou_process_ua	  = 3190;
	instantaneous_power_consumption_ua->multiscan_beidou_capture_ua   = 12600;
	instantaneous_power_consumption_ua->multiscan_beidou_process_ua   = 3430;
	instantaneous_power_consumption_ua->sleep_32k_ua				  = 1210;
	instantaneous_power_consumption_ua->demod_sleep_32m_ua			= 2530;
}

void geolocation_bsp_gnss_prescan_actions( void )
{
	sid_pal_gpio_write(lr_cfg->gpios.gnss_lna, 1);
	sid_pal_gpio_write(lr_cfg->gpios.led_sniff, 1);
	sid_pal_scan_mode(true);
}

void geolocation_bsp_gnss_postscan_actions( void )
{
	sid_pal_gpio_write(lr_cfg->gpios.gnss_lna, 0);
	sid_pal_gpio_write(lr_cfg->gpios.led_sniff, 0);
	sid_pal_scan_mode(false);
}

void geolocation_bsp_wifi_prescan_actions( void )
{
	sid_pal_gpio_write(lr_cfg->gpios.led_sniff, 1);
	sid_pal_scan_mode(true);
}

void geolocation_bsp_wifi_postscan_actions( void )
{
	sid_pal_gpio_write(lr_cfg->gpios.led_sniff, 0);
	sid_pal_scan_mode(false);
}

void geolocation_bsp_get_lr11xx_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode )
{
	*reg_mode = lr_cfg->regulator_mode;
}

lr11xx_system_lfclk_cfg_t geolocation_bsp_get_lr11xx_lf_clock_cfg( )
{
	return lr_cfg->lfclock_cfg;
}

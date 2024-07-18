#include <sx126x.h>
#include <sx126x_hal.h>
#include <sx126x_regs.h>

/**
 * Commands Interface
 */
typedef enum sx126x_commands_e
{
	// Operational Modes Functions
	SX126X_SET_SLEEP				  = 0x84,
	SX126X_SET_STANDBY				= 0x80,
	SX126X_SET_FS					 = 0xC1,
	SX126X_SET_TX					 = 0x83,
	SX126X_SET_RX					 = 0x82,
	SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
	SX126X_SET_RX_DUTY_CYCLE		  = 0x94,
	SX126X_SET_CAD					= 0xC5,
	SX126X_SET_TX_CONTINUOUS_WAVE	 = 0xD1,
	SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
	SX126X_SET_REGULATOR_MODE		 = 0x96,
	SX126X_CALIBRATE				  = 0x89,
	SX126X_CALIBRATE_IMAGE			= 0x98,
	SX126X_SET_PA_CFG				 = 0x95,
	SX126X_SET_RX_TX_FALLBACK_MODE	= 0x93,
	// Registers and buffer Access
	SX126X_WRITE_REGISTER = 0x0D,
	SX126X_READ_REGISTER  = 0x1D,
	SX126X_WRITE_BUFFER   = 0x0E,
	SX126X_READ_BUFFER	= 0x1E,
	// DIO and IRQ Control Functions
	SX126X_SET_DIO_IRQ_PARAMS		 = 0x08,
	SX126X_GET_IRQ_STATUS			 = 0x12,
	SX126X_CLR_IRQ_STATUS			 = 0x02,
	SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
	SX126X_SET_DIO3_AS_TCXO_CTRL	  = 0x97,
	// RF Modulation and Packet-Related Functions
	SX126X_SET_RF_FREQUENCY		  = 0x86,
	SX126X_SET_PKT_TYPE			  = 0x8A,
	SX126X_GET_PKT_TYPE			  = 0x11,
	SX126X_SET_TX_PARAMS			 = 0x8E,
	SX126X_SET_MODULATION_PARAMS	 = 0x8B,
	SX126X_SET_PKT_PARAMS			= 0x8C,
	SX126X_SET_CAD_PARAMS			= 0x88,
	SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
	SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
	// Communication Status Information
	SX126X_GET_STATUS		   = 0xC0,
	SX126X_GET_RX_BUFFER_STATUS = 0x13,
	SX126X_GET_PKT_STATUS	   = 0x14,
	SX126X_GET_RSSI_INST		= 0x15,
	SX126X_GET_STATS			= 0x10,
	SX126X_RESET_STATS		  = 0x00,
	// Miscellaneous
	SX126X_GET_DEVICE_ERRORS = 0x17,
	SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
	// Operational Modes Functions
	SX126X_SIZE_SET_SLEEP				  = 2,
	SX126X_SIZE_SET_STANDBY				= 2,
	SX126X_SIZE_SET_FS					 = 1,
	SX126X_SIZE_SET_TX					 = 4,
	SX126X_SIZE_SET_RX					 = 4,
	SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
	SX126X_SIZE_SET_RX_DUTY_CYCLE		  = 7,
	SX126X_SIZE_SET_CAD					= 1,
	SX126X_SIZE_SET_TX_CONTINUOUS_WAVE	 = 1,
	SX126X_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
	SX126X_SIZE_SET_REGULATOR_MODE		 = 2,
	SX126X_SIZE_CALIBRATE				  = 2,
	SX126X_SIZE_CALIBRATE_IMAGE			= 3,
	SX126X_SIZE_SET_PA_CFG				 = 5,
	SX126X_SIZE_SET_RX_TX_FALLBACK_MODE	= 2,
	// Registers and buffer Access
	// Full size: this value plus buffer size
	SX126X_SIZE_WRITE_REGISTER = 3,
	// Full size: this value plus buffer size
	SX126X_SIZE_READ_REGISTER = 4,
	// Full size: this value plus buffer size
	SX126X_SIZE_WRITE_BUFFER = 2,
	// Full size: this value plus buffer size
	SX126X_SIZE_READ_BUFFER = 3,
	// DIO and IRQ Control Functions
	SX126X_SIZE_SET_DIO_IRQ_PARAMS		 = 9,
	SX126X_SIZE_GET_IRQ_STATUS			 = 2,
	SX126X_SIZE_CLR_IRQ_STATUS			 = 3,
	SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
	SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL	  = 5,
	// RF Modulation and Packet-Related Functions
	SX126X_SIZE_SET_RF_FREQUENCY		   = 5,
	SX126X_SIZE_SET_PKT_TYPE			   = 2,
	SX126X_SIZE_GET_PKT_TYPE			   = 2,
	SX126X_SIZE_SET_TX_PARAMS			  = 3,
	SX126X_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
	SX126X_SIZE_SET_MODULATION_PARAMS_LORA = 5,
	SX126X_SIZE_SET_PKT_PARAMS_GFSK		= 10,
	SX126X_SIZE_SET_PKT_PARAMS_LORA		= 7,
	SX126X_SIZE_SET_CAD_PARAMS			 = 8,
	SX126X_SIZE_SET_BUFFER_BASE_ADDRESS	= 3,
	SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
	// Communication Status Information
	SX126X_SIZE_GET_STATUS		   = 2,
	SX126X_SIZE_GET_RX_BUFFER_STATUS = 2,
	SX126X_SIZE_GET_PKT_STATUS	   = 2,
	SX126X_SIZE_GET_RSSI_INST		= 2,
	SX126X_SIZE_GET_STATS			= 2,
	SX126X_SIZE_RESET_STATS		  = 7,
	// Miscellaneous
	SX126X_SIZE_GET_DEVICE_ERRORS = 2,
	SX126X_SIZE_CLR_DEVICE_ERRORS = 3,
	SX126X_SIZE_MAX_BUFFER		= 255,
	SX126X_SIZE_DUMMY_BYTE		= 1,
} sx126x_commands_size_t;

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_RTC_FREQ_IN_HZ 64000UL

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_XTAL_FREQ 32000000UL

/**
 * @brief Scaling factor used to perform fixed-point operations
 */
#define SX126X_PLL_STEP_SHIFT_AMOUNT ( 14 )

/**
 * @brief PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
 */
#define SX126X_PLL_STEP_SCALED ( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )

sx126x_status_t sx126x_handle_rx_done( const void* context )
{
	return sx126x_stop_rtc( context );
}

uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms )
{
	return ( uint32_t )( timeout_in_ms * ( SX126X_RTC_FREQ_IN_HZ / 1000 ) );
}

sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
												const uint8_t rx_base_address )
{
	const uint8_t buf[SX126X_SIZE_SET_BUFFER_BASE_ADDRESS] = {
		SX126X_SET_BUFFER_BASE_ADDRESS,
		tx_base_address,
		rx_base_address,
	};

	return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_BUFFER_BASE_ADDRESS, 0, 0 );
}

sx126x_status_t sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma )
{
	return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_OCP, &ocp_in_step_of_2_5_ma, 1 );
}

sx126x_status_t sx126x_cfg_tx_clamp( const void* context )
{
	sx126x_status_t status	= SX126X_STATUS_ERROR;
	uint8_t		 reg_value = 0x00;

	status = sx126x_read_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );

	if( status == SX126X_STATUS_OK )
	{
		reg_value |= SX126X_REG_TX_CLAMP_CFG_MASK;
		status = sx126x_write_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );
	}

	return status;
}

sx126x_status_t sx126x_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
	// Perform a floor() to get a value for freq1 corresponding to a frequency lower than or equal to freq1_in_mhz
	const uint8_t freq1 = freq1_in_mhz / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

	// Perform a ceil() to get a value for freq2 corresponding to a frequency higher than or equal to freq2_in_mhz
	const uint8_t freq2 =
		( freq2_in_mhz + SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ - 1 ) / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

	return sx126x_cal_img( context, freq1, freq2 );
}

sx126x_status_t sx126x_set_tx_infinite_preamble( const void* context )
{
	const uint8_t buf[SX126X_SIZE_SET_TX_INFINITE_PREAMBLE] = {
		SX126X_SET_TX_INFINITE_PREAMBLE,
	};

	return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_INFINITE_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_stop_timer_on_preamble( const void* context, const bool enable )
{
	const uint8_t buf[SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE] = {
		SX126X_SET_STOP_TIMER_ON_PREAMBLE,
		( enable == true ) ? 1 : 0,
	};

	return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
	const uint8_t buf[SX126X_SIZE_SET_RX] = {
		SX126X_SET_RX,
		( uint8_t )( timeout_in_rtc_step >> 16 ),
		( uint8_t )( timeout_in_rtc_step >> 8 ),
		( uint8_t )( timeout_in_rtc_step >> 0 ),
	};

	return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX, 0, 0 );
}

sx126x_status_t sx126x_init_retention_list( const void* context )
{
	const uint16_t list_of_registers[3] = { SX126X_REG_RXGAIN, SX126X_REG_TX_MODULATION, SX126X_REG_IQ_POLARITY };

	return sx126x_add_registers_to_retention_list( context, list_of_registers,
												   sizeof( list_of_registers ) / sizeof( list_of_registers[0] ) );
}

uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz )
{
	uint32_t steps_int;
	uint32_t steps_frac;

	// Get integer and fractional parts of the frequency computed with a PLL step scaled value
	steps_int  = freq_in_hz / SX126X_PLL_STEP_SCALED;
	steps_frac = freq_in_hz - ( steps_int * SX126X_PLL_STEP_SCALED );

	// Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
	return ( steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT ) +
		   ( ( ( steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
			 SX126X_PLL_STEP_SCALED );
}

sx126x_status_t sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
													  const uint8_t trimming_cap_xtb )
{
	uint8_t trimming_capacitor_values[2] = { trimming_cap_xta, trimming_cap_xtb };

	return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_XTATRIM, trimming_capacitor_values, 2 );
}

sx126x_status_t sx126x_add_registers_to_retention_list( const void* context, const uint16_t* register_addr,
														uint8_t register_nb )
{
	sx126x_status_t status = SX126X_STATUS_ERROR;
	uint8_t		 buffer[9];

	status = sx126x_read_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );

	if( status == SX126X_STATUS_OK )
	{
		const uint8_t initial_nb_of_registers = buffer[0];
		uint8_t*	  register_list		   = &buffer[1];

		for( uint8_t index = 0; index < register_nb; index++ )
		{
			bool register_has_to_be_added = true;

			// Check if the current register is already added to the list
			for( uint8_t i = 0; i < buffer[0]; i++ )
			{
				if( register_addr[index] == ( ( uint16_t ) register_list[2 * i] << 8 ) + register_list[2 * i + 1] )
				{
					register_has_to_be_added = false;
					break;
				}
			}

			if( register_has_to_be_added == true )
			{
				if( buffer[0] < SX126X_MAX_NB_REG_IN_RETENTION )
				{
					register_list[2 * buffer[0]]	 = ( uint8_t )( register_addr[index] >> 8 );
					register_list[2 * buffer[0] + 1] = ( uint8_t )( register_addr[index] >> 0 );
					buffer[0] += 1;
				}
				else
				{
					return SX126X_STATUS_ERROR;
				}
			}
		}

		if( buffer[0] != initial_nb_of_registers )
		{
			status = sx126x_write_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );
		}
	}

	return status;
}

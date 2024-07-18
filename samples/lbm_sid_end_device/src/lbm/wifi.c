#include <sidewalk.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi, CONFIG_LBM_LOG_LEVEL);

smtc_modem_wifi_event_data_scan_done_t					  wifi_scan_done_data;

/**
 * @brief Wi-Fi payload format (as defined by LR1110 WiFi positioning protocol of LoRaCloud).
 */
typedef enum wifi_mw_payload_format_e
{
	WIFI_MW_PAYLOAD_MAC			= 0x00,  //!< Only the MAC addresses of the detected Access Points are sent
	WIFI_MW_PAYLOAD_MAC_RSSI	= 0x01,  //!< Both MAC address and RSSI of detected Access Points are sent
} wifi_mw_payload_format_t;

/**
 * @brief Size in bytes of the payload tag to indicate frame format (as defined by LR1110 WiFi positioning protocol of
 * LoRaCloud)
 */
#define WIFI_TAG_SIZE ( 1 )

/**
 * @brief Minimal number of detected access point in a scan result to consider the scan valid
 */
#define WIFI_SCAN_NB_AP_MIN ( 3 )

/**
 * @brief Size in bytes to store the RSSI of a detected WiFi Access-Point
 */
#define WIFI_AP_RSSI_SIZE ( 1 )

/*!
 * @brief The format of the Wi-Fi scan results to be used.
 */
static wifi_mw_payload_format_t payload_format = WIFI_MW_PAYLOAD_MAC;

/*!
 * @brief The buffer containing results to be sent over the air
 */
static uint8_t wifi_result_buffer[WIFI_TAG_SIZE + ( ( WIFI_AP_RSSI_SIZE + WIFI_AP_ADDRESS_SIZE ) * WIFI_MAX_RESULTS )];
static uint8_t wifi_buffer_size;

int wifi_uplink()
{
	return uplink_scan(TLV_TAG_WIFI_SCAN, wifi_result_buffer, wifi_buffer_size);
}

int wifi_send_results()
{
	wifi_buffer_size = 0;
	/* Check if there are results to be sent */
	if (wifi_scan_done_data.nbr_results < WIFI_SCAN_NB_AP_MIN) {
		LOG_WRN("only %u results", wifi_scan_done_data.nbr_results);
		return -1;
	}

	/* Add the payload format tag */
	wifi_result_buffer[wifi_buffer_size] = payload_format;
	wifi_buffer_size += WIFI_TAG_SIZE;

	/* Concatenate all results in send buffer */
	for( uint8_t i = 0; i < wifi_scan_done_data.nbr_results; i++ )
	{
		/* Copy Access Point RSSI address in result buffer (if requested) */
		if( payload_format == WIFI_MW_PAYLOAD_MAC_RSSI )
		{
			wifi_result_buffer[wifi_buffer_size] = wifi_scan_done_data.results[i].rssi;
			wifi_buffer_size += WIFI_AP_RSSI_SIZE;
		}
		/* Copy Access Point MAC address in result buffer */
		memcpy( &wifi_result_buffer[wifi_buffer_size], wifi_scan_done_data.results[i].mac_address, WIFI_AP_ADDRESS_SIZE );
		wifi_buffer_size += WIFI_AP_ADDRESS_SIZE;
	}
	//LOG_INF("wifi_buffer_size %u", wifi_buffer_size);
	LOG_HEXDUMP_INF(wifi_result_buffer, wifi_buffer_size, "wifi_result_buffer");

	return wifi_uplink();
}

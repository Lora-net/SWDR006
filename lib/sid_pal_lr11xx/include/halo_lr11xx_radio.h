/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file has private functions needed by the driver
 */

#ifndef HALO_LR11XX_RADIO_H
#define HALO_LR11XX_RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include <sid_pal_serial_bus_ifc.h>
#include <lr11xx_config.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_gpio_ifc.h>

#include "lr11xx_system.h"

#define LP_MAX_POWER                       14
#define LP_MIN_POWER                       -17
#define HP_MAX_POWER                       22
#define HP_MIN_POWER                       -9

/* sid_pal_radio_ifc.h defines all SID_PAL_RADIO_* */
#define	SID_PAL_RADIO_SCAN	   (SID_PAL_RADIO_BUSY + 1)

/*
 * @brief RADIO FSK status enumeration definition
 */
typedef enum {
    RADIO_FSK_RX_DONE_STATUS_OK                  = 0,
    RADIO_FSK_RX_DONE_STATUS_INVALID_PARAMETER   = 1,
    RADIO_FSK_RX_DONE_STATUS_INVALID_LENGTH      = 2,
    RADIO_FSK_RX_DONE_STATUS_BAD_CRC             = 3,
    RADIO_FSK_RX_DONE_STATUS_TIMEOUT             = 4,
    RADIO_FSK_RX_DONE_STATUS_UNKNOWN_ERROR       = 5,
    RADIO_FSK_RX_DONE_STATUS_SW_MARK_NOT_PRESENT = 6,
} radio_fsk_rx_done_status_t;

typedef struct {
    const radio_lr11xx_device_config_t           *config;
    const struct sid_pal_serial_bus_iface        *bus_iface;

    sid_pal_radio_modem_mode_t                   modem;
    sid_pal_radio_rx_packet_t                    *radio_rx_packet;
    sid_pal_radio_event_notify_t                 report_radio_event;
    uint8_t                                      radio_state;
    sid_pal_radio_irq_handler_t                  irq_handler;

    sid_pal_radio_cad_param_exit_mode_t          cad_exit_mode;
    radio_lr11xx_pa_cfg_t                        pa_cfg;
#if HALO_ENABLE_DIAGNOSTICS
    bool                                         pa_cfg_configured;
#endif
	bool										 rx_timeout_sim;
	bool										 sleeping;
	bool										inhibit_sleep;
	bool										post_scan_recovery;

	struct {
		int						csuso;
		struct sid_timespec      rcv_tm;
		bool					timer_armed;
		unsigned                lora_timeout_us;
	} suppress_rx_timeout;

    lr11xx_system_irq_mask_t                     irq_mask;
    uint32_t                                     radio_freq_hz;

    lr11xx_system_version_t                      ver;

    struct {
        sid_pal_radio_lora_packet_params_t       lora_pkt_params;
        sid_pal_radio_lora_modulation_params_t   lora_mod_params;
        sid_pal_radio_fsk_packet_params_t        fsk_pkt_params;
        sid_pal_radio_fsk_modulation_params_t    fsk_mod_params;
        sid_pal_radio_fsk_cad_params_t           fsk_cad_params;
    }                                            settings_cache;

    struct {
        uint8_t                                  stat1;
        uint8_t                                  stat2;
        uint16_t                                 command;
		uint16_t								 failedCommand;
    }                                            last;
    radio_lr11xx_regional_param_t                regional_radio_param;
} halo_drv_semtech_ctx_t;

#define LR11XX_TUS_IN_SEC                           32768UL
#define US_IN_SEC                                   (1000000UL)
#define LR11XX_US_TO_SYMBOLS(time_in_us, bit_rate)  ((uint64_t)((uint64_t)time_in_us * (uint64_t)bit_rate) / US_IN_SEC)
#define LR11XX_TUS_TO_US(X)                         ((X) * US_IN_SEC / LR11XX_TUS_IN_SEC)
#define NOT_FULL_FUNCTIONING_VERSION                0x0303

halo_drv_semtech_ctx_t* lr11xx_get_drv_ctx(void);

int32_t radio_lora_process_rx_done(halo_drv_semtech_ctx_t *drv_ctx);

int32_t radio_fsk_process_sync_word_detected(halo_drv_semtech_ctx_t *drv_ctx);

int32_t radio_fsk_process_rx_done(halo_drv_semtech_ctx_t *drv_ctx, radio_fsk_rx_done_status_t *rx_done_status);

void set_lora_exit_mode(sid_pal_radio_cad_param_exit_mode_t cad_exit_mode);

int32_t set_radio_int(const halo_drv_semtech_ctx_t *drv_ctx, bool int_enable);

int32_t radio_disable_irq(const halo_drv_semtech_ctx_t* drv);

int32_t radio_enable_irq(const halo_drv_semtech_ctx_t* drv);

int32_t get_radio_lr11xx_pa_config(radio_lr11xx_pa_cfg_t *cfg);

void sid_pal_scan_mode(bool);
void lr11xx_restore_transceiver(void);

#ifdef __cplusplus
}
#endif

#endif /* HALO_LR11XX_RADIO_H */

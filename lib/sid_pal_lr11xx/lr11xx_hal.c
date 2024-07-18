/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file defines interface used by Semtech driver to perform platform specific
 * operations
 */

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <sid_pal_delay_ifc.h>

#include "halo_lr11xx_radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_hal.h"

// #define LOCAL_DEBUG 1
//#include <sid_hal_log.h>
#include <zephyr/logging/log.h>

#define SEMTECH_BOOTUP_DELAY_US			185000
#define STATUS_FIELD_OFFSET_BITS		   1
#define STATUS_OK_MASK					 (LR11XX_SYSTEM_CMD_STATUS_OK << STATUS_FIELD_OFFSET_BITS)
#define DEFAULT_WAKEUP_DELAY			   97
// Delay time to allow for any external PA/FEM turn ON/OFF
#define SEMTECH_STDBY_STATE_DELAY_US	   10
#define SEMTECH_MAX_WAIT_ON_BUSY_CNT_US	40000
#define SEMTECH_BOOTLOADER_MAX_WAIT_ON_BUSY_CNT_US	400000

LOG_MODULE_REGISTER(lr11xx_hal, CONFIG_SIDEWALK_LOG_LEVEL);

static sid_error_t lr11xx_wait_on_busy(const halo_drv_semtech_ctx_t *drv_ctx, unsigned max_wait_us)
{
	assert(drv_ctx);

	uint8_t is_radio_busy = 0;
	uint16_t cnt = 0;
	sid_error_t err = SID_ERROR_NONE;

	while (cnt++ < max_wait_us) {
		err = sid_pal_gpio_read(drv_ctx->config->gpios.radio_busy, &is_radio_busy);
		if ((err == SID_ERROR_NONE) && !is_radio_busy) {
		   break;
		}
		sid_pal_delay_us(SEMTECH_STDBY_STATE_DELAY_US);
	}

	if (cnt >= max_wait_us) {
		return SID_ERROR_BUSY;
	}
	return SID_ERROR_NONE;
}

static lr11xx_hal_status_t lr11xx_hal_rdwr(const halo_drv_semtech_ctx_t* cctx,
										   const uint8_t* command,
										   const uint16_t command_length,
										   uint8_t* data,
										   const uint16_t data_length,
										   bool read)
{
	unsigned max_wait_us;
	halo_drv_semtech_ctx_t* ctx = (halo_drv_semtech_ctx_t*) cctx;
	assert(ctx);

	if (cctx->sleeping) {
		lr11xx_hal_status_t err;
		if ((err = lr11xx_hal_wakeup(cctx)) != LR11XX_HAL_STATUS_OK) {
			LOG_ERR("fail wakeup %d", err);
		}
	}

	if (command[0] == 0x80) /* if flash write in bootloader */
		max_wait_us = SEMTECH_BOOTLOADER_MAX_WAIT_ON_BUSY_CNT_US;
	else
		max_wait_us = SEMTECH_MAX_WAIT_ON_BUSY_CNT_US;

	if (lr11xx_wait_on_busy(ctx, max_wait_us) != SID_ERROR_NONE) {
		LOG_ERR("rdwr start wait_on_busy %02x%02x, waited %u", command[0], command[1], max_wait_us);
		return LR11XX_HAL_STATUS_ERROR;
	}

#ifdef LOCAL_DEBUG
	SID_HAL_LOG_INFO("-----------------------------");
	SID_HAL_LOG_INFO((read)? "Command (read):" : "Command (write):");
	SID_HAL_LOG_HEXDUMP_INFO(command, command_length);
	if (!read && data_length > 0) {
		SID_HAL_LOG_INFO("Data:");
		SID_HAL_LOG_HEXDUMP_INFO(data, data_length);
	}
#endif

	size_t size = command_length;
	uint8_t *buff = ctx->config->internal_buffer.p;
	memcpy(buff, command, command_length);
	if (!read && data_length > 0) {
		size += data_length;
		memcpy(&buff[command_length], data, data_length);
	}

	int err = ctx->bus_iface->xfer(ctx->bus_iface, &ctx->config->bus_selector, buff, buff, size);
	if (err != SID_ERROR_NONE) {
		LOG_ERR("rdwr xfer write");
		return LR11XX_HAL_STATUS_ERROR;
	}

	ctx->last.stat1 = buff[0];
	ctx->last.stat2 = buff[1];
	if (!(ctx->last.stat1 & STATUS_OK_MASK) && ctx->last.command) {
		/* section 3.4.2: bit0 = interrupt status */
		LOG_WRN("during %02x%02x, Command 0x%.4X failed; Stat1 0x%.2X", command[0], command[1], ctx->last.command, ctx->last.stat1);
		ctx->last.failedCommand = ctx->last.command;
	}

	ctx->last.command = (command[0] << 8) | command[1];
	if (ctx->last.command == 0x11b)
		ctx->sleeping = true;

#ifdef LOCAL_DEBUG
	SID_HAL_LOG_INFO("Read back");
	SID_HAL_LOG_HEXDUMP_INFO(empty, size);
#endif

	if (!read) {
		return LR11XX_HAL_STATUS_OK;
	}

	if (lr11xx_wait_on_busy(ctx, SEMTECH_MAX_WAIT_ON_BUSY_CNT_US) != SID_ERROR_NONE) {
		LOG_ERR("rdwr busy read");
		return LR11XX_HAL_STATUS_ERROR;
	}

	size = data_length + 1;
	buff = ctx->config->internal_buffer.p;
	memset(buff, 0, size);
	err = ctx->bus_iface->xfer(ctx->bus_iface, &ctx->config->bus_selector, buff, buff, size);
	if (err != SID_ERROR_NONE) {
		LOG_ERR("rdwr xfer read");
		return LR11XX_HAL_STATUS_ERROR;
	}

	ctx->last.stat1 = buff[0];
	if (!(ctx->last.stat1 & STATUS_OK_MASK) && ctx->last.command) {
		LOG_WRN("LR1110: Command rsp 0x%.4X failed; Stat1 0x%.2X", ctx->last.command, ctx->last.stat1);
	}

#ifdef LOCAL_DEBUG
	SID_HAL_LOG_INFO("Data");
	SID_HAL_LOG_HEXDUMP_INFO(buff, size);
#endif

	memcpy(data, &buff[1], data_length);

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
	halo_drv_semtech_ctx_t* ctx = (halo_drv_semtech_ctx_t*) context;
	const radio_lr11xx_device_config_t *config = ctx->config;
	uint8_t command[4] = { 0 };

	int err = ctx->bus_iface->xfer(ctx->bus_iface, &config->bus_selector, command, NULL, sizeof(command));
	if (err != SID_ERROR_NONE) {
		LOG_ERR("abort_blocking xfer");
		return LR11XX_STATUS_ERROR;
	}
	if (lr11xx_wait_on_busy(ctx, 1000000) != SID_ERROR_NONE) {
		LOG_ERR("abort_blocking wait on busy");
		return LR11XX_STATUS_ERROR;
	}
	return LR11XX_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context,
											uint8_t* data,
											const uint16_t data_length )
{
	halo_drv_semtech_ctx_t* ctx = (halo_drv_semtech_ctx_t*) context;
	assert(ctx);
	assert(data);
	assert(data_length != 0);
	assert(data_length <= ctx->config->internal_buffer.size);

	const radio_lr11xx_device_config_t *config = ctx->config;
	uint8_t *empty = config->internal_buffer.p;
	memset(empty, 0, data_length);

	if (ctx->sleeping) {
		lr11xx_hal_status_t err;
		LOG_WRN("direct_read self-wakeup");
		if ((err = lr11xx_hal_wakeup(ctx)) != LR11XX_HAL_STATUS_OK) {
			LOG_ERR("fail wakeup %d", err);
		}
	}

	if (lr11xx_wait_on_busy(ctx, SEMTECH_MAX_WAIT_ON_BUSY_CNT_US) != SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	int err = ctx->bus_iface->xfer(ctx->bus_iface, &config->bus_selector, empty, data, data_length);

	if (!(ctx->last.stat1 & STATUS_OK_MASK) && ctx->last.command) {
		LOG_WRN("LR1110: Command 0x%.4X failed; Stat1 0x%.2X", ctx->last.command, ctx->last.stat1);
	}
	ctx->last.command = 0; // No command  - only read. Chip will complain about it

#ifdef LOCAL_DEBUG
	SID_HAL_LOG_INFO("-----------------------------");
	SID_HAL_LOG_INFO("Direct read");
	SID_HAL_LOG_HEXDUMP_INFO(data, data_length);
#endif

	return (err == SID_ERROR_NONE)? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_reset(const void *ctx)
{
	if (NULL == ctx) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	const halo_drv_semtech_ctx_t *drv_ctx = (halo_drv_semtech_ctx_t *)ctx;

	if (sid_pal_gpio_set_direction(drv_ctx->config->gpios.power,
		SID_PAL_GPIO_DIRECTION_OUTPUT) != SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (sid_pal_gpio_output_mode(drv_ctx->config->gpios.power,
		SID_PAL_GPIO_OUTPUT_PUSH_PULL) != SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (sid_pal_gpio_write(drv_ctx->config->gpios.power, 0) != SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	sid_pal_delay_us(SEMTECH_BOOTUP_DELAY_US);
	if (sid_pal_gpio_write(drv_ctx->config->gpios.power, 1) != SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	sid_pal_delay_us(SEMTECH_BOOTUP_DELAY_US);

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
	halo_drv_semtech_ctx_t *drv_ctx = (halo_drv_semtech_ctx_t *)context;

	if (drv_ctx == NULL) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (drv_ctx->radio_state != SID_PAL_RADIO_SLEEP && !drv_ctx->sleeping) {
		LOG_WRN("!SID_PAL_RADIO_SLEEP and !sleeping");
		return LR11XX_HAL_STATUS_OK;
	}

	if (sid_pal_gpio_set_direction(drv_ctx->config->bus_selector.client_selector,
		SID_PAL_GPIO_DIRECTION_OUTPUT) != SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (sid_pal_gpio_write(drv_ctx->config->bus_selector.client_selector, 0)
		!= SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	sid_pal_delay_us(drv_ctx->config->wakeup_delay_us? drv_ctx->config->wakeup_delay_us : DEFAULT_WAKEUP_DELAY);

	/* pull up NSS pin again to allow transactions */
	if (sid_pal_gpio_write(drv_ctx->config->bus_selector.client_selector, 1)
		!= SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	/* Wait for chip to be ready */
	if (lr11xx_wait_on_busy(drv_ctx, SEMTECH_MAX_WAIT_ON_BUSY_CNT_US) != SID_ERROR_NONE) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	drv_ctx->sleeping = false;

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
									 uint8_t* data, const uint16_t data_length)
{
	if ( context == NULL || command == NULL || data == NULL || command_length == 0 || data_length == 0) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	return lr11xx_hal_rdwr(context, command, command_length, data, data_length, true);
}

static inline bool is_scan_start_command(const uint8_t* command)
{
	if (command[0] == 0x04) { // gnss
		if (command[1] == 0x09) // autonomous start
			return true;
		else if (command[1] == 0x0a) // assisted start
			return true;
		else
			return false;
	} else if (command[0] == 0x03) { // wifi
		if (command[1] == 0x00) // scan start
			return true;
		else
			return false;
	} else
		return false;
}

lr11xx_hal_status_t lr11xx_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
									 const uint8_t* data, const uint16_t data_length)
{
	lr11xx_hal_status_t ret;
	/* For write data can be null and data length 0 */
	if ( context == NULL || command == NULL || command_length == 0) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	ret = lr11xx_hal_rdwr(context, command, command_length, (void*)data, data_length, false);
	if (ret == LR11XX_HAL_STATUS_OK && is_scan_start_command(command)) {
		const halo_drv_semtech_ctx_t *cctx = context;
		if (cctx->radio_state != SID_PAL_RADIO_SCAN) {
			/* sid_pal_scan_mode() should have been called */
			LOG_ERR("scan start cmd while not in scan state");
		}
	}
	return ret;
}

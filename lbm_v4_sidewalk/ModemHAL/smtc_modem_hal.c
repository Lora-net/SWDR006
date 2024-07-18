/*********************************************************************
* COPYRIGHT 2022 CONNECTED DEVELOPMENT, A DIVISION OF EXPONENTIAL
* TECHNOLOGY GROUP.
*
* SPDX-License-Identifier: Apache-2.0
*
*******************************************************************************
* @file
* @brief  Connected Development implementation of the Modem Hardware Abstraction
*		 Layer for the Connected Development SX1262 shield.
*
* @details  Copied from the example in the Semtech LoRa Basics Modem SDK at:
*		   SWSD001\smtc_hal\STMicroelectronics\STM32L4xx\smtc_modem_hal\smtc_modem_hal.c
******************************************************************************/

/*!
 * \file	  smtc_modem_hal.c
 *
 * \brief	 Modem Hardware Abstraction Layer API implementation.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the Semtech corporation nor the
 *	   names of its contributors may be used to endorse or promote products
 *	   derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// for variadic args
#include <stdio.h>
#include <stdarg.h>

// for memcpy
#include <string.h>

#include <zephyr/settings/settings.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <nrfx_gpiote.h>

#ifndef LR11XX
#include "ral_sx126x_bsp.h"
#include "ralf_sx126x.h"
#include "sx126x_hal_context.h"
#endif /* !LR11XX */
#include "smtc_modem_hal.h"
#include "smtc_modem_hal_dbg_trace.h"

#include <zephyr/logging/log_ctrl.h>	// LOG_PANIC
#include <zephyr/logging/log.h>

#ifdef SMTC_MODEM_HAL_IRQ_FROM_SID_PAL
#include <halo_lr11xx_radio.h>
#else
#include <sid_pal_gpio_ifc.h>
#endif /* SMTC_MODEM_HAL_IRQ_FROM_SID_PAL */
#include <app_subGHz_config.h>
#ifdef PORTING_TEST
LOG_MODULE_REGISTER(ModemHAL, CONFIG_SIDEWALK_LOG_LEVEL);
#else
LOG_MODULE_REGISTER(ModemHAL, CONFIG_LBM_LOG_LEVEL);
#endif

void app_event_lbm(void); // run app-layer once for LBM engine
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define SETTINGS_SUBTREE_NAME				"lbm_context"
//#define ADDR_FLASH_MODEM_CONTEXT			 SETTINGS_SUBTREE_NAME "/modem"
//#define ADDR_FLASH_LR1MAC_CONTEXT			SETTINGS_SUBTREE_NAME "/lr1mac"
//#define ADDR_FLASH_DEVNONCE_CONTEXT		  SETTINGS_SUBTREE_NAME "/devnonce"
//#define ADDR_FLASH_SECURE_ELEMENT_CONTEXT	SETTINGS_SUBTREE_NAME "/secure_element"
#define ADDR_FLASH_MODEM_CONTEXT			 SETTINGS_SUBTREE_NAME "/modem"
#define ADDR_FLASH_MODEM_KEY_CONTEXT		 SETTINGS_SUBTREE_NAME "/key"
#define ADDR_FLASH_LORAWAN_CONTEXT		   SETTINGS_SUBTREE_NAME "/lorawan"
#define ADDR_FLASH_FUOTA					 SETTINGS_SUBTREE_NAME "/fuota"
#define ADDR_FLASH_SECURE_ELEMENT_CONTEXT	SETTINGS_SUBTREE_NAME "/secure_element"
#define ADDR_FLASH_STORE_AND_FORWARD		 SETTINGS_SUBTREE_NAME "/store_and_forward"


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct DirectLoadParams_s
{
   size_t   len;
   void	 *dest;
   bool	 fetched;
} DirectLoadParams_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

__attribute__( ( section( ".noinit" ) ) ) static uint8_t		  crashlog_buff_noinit[CRASH_LOG_SIZE];
__attribute__( ( section( ".noinit" ) ) ) static volatile uint8_t crashlog_length_noinit;
__attribute__( ( section( ".noinit" ) ) ) static volatile bool	crashlog_available_noinit;

static struct k_timer		 halTimer;
static volatile bool		  halTimerIrqEnabled = true;
static volatile bool		  halTimerIrqPending = false;
static bool				   settingsSubsysInit = false;
static void				   (*HalTimerCallback)(void *context);
static void				   *halTimerContext;

static void				   (*HalDio1Callback)(void *context);
static void				   *halDio1Context;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void HalTimerHandler(struct k_timer *timer);
static bool InitSettingsSubsys(void);
static int DirectLoadHandler(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg, void *param);

static void HalTimerWorkHandler(struct k_work *work);
static K_WORK_DEFINE(halTimerWorkItem, HalTimerWorkHandler);
volatile bool smtc_modem_hal_timer_running;

static void HalDio1WorkHandler(struct k_work *work);
static K_WORK_DEFINE(halDio1WorkItem, HalDio1WorkHandler);

static void TemperatureGet(struct sensor_value *temperature);


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* ------------ Reset management ------------*/

/**
 * @brief Resets the MCU.
 */
void smtc_modem_hal_reset_mcu(void)
{
#ifdef LBM_REBOOT_ON_PANIC
	k_sleep(K_MSEC(2000));
	NVIC_SystemReset(); // release build
#else
    /* Flush the logs before locking the CPU */
    LOG_PANIC();

    while (true) {
    };
#endif
}

/* ------------ Watchdog management ------------*/

/**
 * @brief Reloads watchdog counter.
 *
 * @remark Application has to call this function periodically.
 *		 The call period must be less than WATCHDOG_RELOAD_PERIOD.
 */
void smtc_modem_hal_reload_wdog(void)
{
   // @todo - Implement this if using watchdog.
   // Example in nrf_desktop sample application, release build.
//   int err = wdt_feed(data->wdt, data->wdt_channel_id);
}

/* ------------ Time management ------------*/

/**
 * @brief Returns the current time in seconds.
 *
 * @remark Used for scheduling autonomous retransmissions (i.e: NbTrans),
 *		 transmitting MAC answers, basically any delay without accurate time
 *		 constraints. It is also used to measure the time spent inside the
 *		 LoRaWAN process for the integrated failsafe.
 *
 * @return uint32_t Current time in seconds.
 */
uint32_t smtc_modem_hal_get_time_in_s(void)
{
   int64_t uptime64ms = k_uptime_get();
   return (uint32_t) (uptime64ms / 1000);
}

/**
 * @brief Returns the compensated current time in seconds.
 *
 * @remark Used for Clock synchronization process ALCSync which need an
 *		 accurate clock with compensated drift.
 *
 * @return uint32_t Current time in seconds.
 */
/*uint32_t smtc_modem_hal_get_compensated_time_in_s(void)
{
   return smtc_modem_hal_get_time_in_s() + smtc_modem_hal_get_time_compensation_in_s();
}*/

/**
 * @brief Returns the time compensation in seconds.
 *
 * @remark Used for Clock synchronization process ALCSync which need an
 *		 accurate clock with compensated drift.
 *
 * @return int32_t the positive or negative compensation offset in seconds.
 */
int32_t smtc_modem_hal_get_time_compensation_in_s(void)
{
   return 0;
}

/**
 * @brief Returns the current time in milliseconds.
 *
 *
 * @return uint32_t Current time in milliseconds (wraps every 49 days).
 */
uint32_t smtc_modem_hal_get_time_in_ms(void)
{
   return (uint32_t) k_uptime_get();
}

/**
 * @brief Returns the current time in 0.1 milliseconds.
 *
 * @remark Used for class B ping slot openings. Must be the same timer
 * as the one used for \ref smtc_modem_hal_get_radio_irq_timestamp_in_100us.
 *
 * @return uint32_t Current time in 100µs (wraps every 4,9 days).
 */
uint32_t smtc_modem_hal_get_time_in_100us(void)
{
   int64_t uptime64us = k_ticks_to_us_floor64(k_uptime_ticks());
   return (uint32_t) (uptime64us / 100);
}

/**
 * @brief Returns the time, in 0.1 milliseconds, of the last radio interrupt request.
 *
 *  @remark Used to obtain the timestamp of radio events (i.e.: end of TX).
 *  Must be the same timer as the one used for \ref smtc_modem_hal_get_time_in_100us.
 *
 * @return uint32_t
 */
uint32_t smtc_modem_hal_get_radio_irq_timestamp_in_100us(void)
{
   // In current LBM implementation the call of this function is done in radio_planner radio irq handler
   // so the current time is the irq time
   return smtc_modem_hal_get_time_in_100us();
}

/* ------------ Timer management ------------*/

/**
 * @brief Starts the provided timer object for the given time.
 *
 * @param [in] milliseconds Number of milliseconds (timer value).
 * @param [in] callback	 Callback that will be called in case of timer irq.
 * @param [in] context	  Context that will be passed on callback argument.
 */
void smtc_modem_hal_start_timer(const uint32_t milliseconds, void(*callback)(void *context), void *context)
{
	static bool initialized = false;
	LOG_DBG("Start timer for %u msec.", milliseconds);
	if (!initialized) {
		k_timer_init(&halTimer, HalTimerHandler, NULL);
		initialized = true;
	}
	HalTimerCallback = callback;
	halTimerContext = context;
	if (milliseconds < 5)
		k_timer_start(&halTimer, K_MSEC(milliseconds+5), K_NO_WAIT);	// TODO reasonable minimal value
	else
		k_timer_start(&halTimer, K_MSEC(milliseconds), K_NO_WAIT);
	smtc_modem_hal_timer_running = true;
}

/**
 * @brief Stop the provided timer.
 */
void smtc_modem_hal_stop_timer(void)
{
   LOG_DBG("Stop timer.");

   k_timer_stop(&halTimer);
   HalTimerCallback = NULL;
   halTimerContext = NULL;
}

/* ------------ IRQ management ------------*/

/**
 * @brief Disables interruptions used in Modem (radio_dio and timer).
 */
void smtc_modem_hal_disable_modem_irq(void)
{
#ifndef SMTC_MODEM_HAL_IRQ_FROM_SID_PAL
	sid_error_t err;
	LOG_DBG("Disable DIO1 IRQ.");

#ifdef LR11XX
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	if ((err = sid_pal_gpio_irq_disable(cfg->gpios.int1)) != SID_ERROR_NONE) {
		LOG_ERR("%d = sid_pal_gpio_irq_disable()", err);
	}
#else
	const radio_sx126x_device_config_t *cfg = get_radio_cfg();
	sid_pal_gpio_irq_disable(cfg->gpio_int1);
#endif

#endif /* !SMTC_MODEM_HAL_IRQ_FROM_SID_PAL */
	halTimerIrqEnabled = false;
}

/**
 * @brief Enables interruptions used in Modem (radio_dio and timer).
 */
void smtc_modem_hal_enable_modem_irq(void)
{
#ifndef SMTC_MODEM_HAL_IRQ_FROM_SID_PAL
	LOG_DBG("Enable DIO1 IRQ.");

#ifdef LR11XX
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	sid_pal_gpio_irq_enable(cfg->gpios.int1);
#else
	const radio_sx126x_device_config_t *cfg = get_radio_cfg();
	sid_pal_gpio_irq_enable(cfg->gpio_int1);
#endif

#endif /* !SMTC_MODEM_HAL_IRQ_FROM_SID_PAL */
	halTimerIrqEnabled = true;
	if (halTimerIrqPending) {
		k_work_submit(&halTimerWorkItem);
		halTimerIrqPending = false;
	}
}

/* ------------ Context saving management ------------*/

/**
 * @brief Restores the data context.
 *
 * @remark This function is used to restore Modem data from a non-volatile memory.
 *
 * @param [in] ctx_type   Type of modem context that need to be restored.
 * @param [out] buffer	Buffer pointer to write to.
 * @param [in] size	   Buffer size to read in bytes.
 */
void smtc_modem_hal_context_restore(const modem_context_type_t ctx_type, uint32_t offset, uint8_t *buffer, const uint32_t size)
{
   int rc = 0;
   DirectLoadParams_t directLoadParams;

   directLoadParams.fetched = false;
   directLoadParams.len = size;
   directLoadParams.dest = buffer + offset;

   if (InitSettingsSubsys())
   {
	  switch (ctx_type)
	  {
#if 0
		 case CONTEXT_MODEM:
			rc = settings_load_subtree_direct(ADDR_FLASH_MODEM_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;

		 case CONTEXT_LR1MAC:
			rc = settings_load_subtree_direct(ADDR_FLASH_LR1MAC_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;

		 case CONTEXT_DEVNONCE:
			rc = settings_load_subtree_direct(ADDR_FLASH_DEVNONCE_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;

		 case CONTEXT_SECURE_ELEMENT:
			rc = settings_load_subtree_direct(ADDR_FLASH_SECURE_ELEMENT_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;
#endif /* #if 0 */
		case CONTEXT_MODEM:
			rc = settings_load_subtree_direct(ADDR_FLASH_MODEM_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;
		case CONTEXT_KEY_MODEM:
			rc = settings_load_subtree_direct(ADDR_FLASH_MODEM_KEY_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;
		case CONTEXT_LORAWAN_STACK:
			rc = settings_load_subtree_direct(ADDR_FLASH_LORAWAN_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;
		case CONTEXT_FUOTA:
			rc = settings_load_subtree_direct(ADDR_FLASH_FUOTA,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;
		case CONTEXT_SECURE_ELEMENT:
			rc = settings_load_subtree_direct(ADDR_FLASH_SECURE_ELEMENT_CONTEXT,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;
		case CONTEXT_STORE_AND_FORWARD:
			rc = settings_load_subtree_direct(ADDR_FLASH_STORE_AND_FORWARD,
											  DirectLoadHandler,
											  (void *) &directLoadParams);
			break;
			

		 default:
			LOG_ERR("Unknown context: %u", (uint32_t) ctx_type);
			k_sleep(K_MSEC(200));
			k_panic();
			break;
	  }

	  if (rc != 0)
	  {
		 LOG_ERR("Settings Get Context %u. Error=%d", (uint32_t) ctx_type, rc);
	  }
	  else if (!directLoadParams.fetched)
	  {
		 rc = -ENOENT;
	  }
	  else
	  {
		 LOG_DBG("Context %u restored.", (uint32_t) ctx_type);
	  }
   }
}

/**
 * @brief Stores the data context.
 *
 * @remark This function is used to store Modem data in a non-volatile memory.
 *
 * @param [in] ctx_type   Type of modem context that need to be saved.
 * @param [in] buffer	 Buffer pointer to write from.
 * @param [in] size	   Buffer size to write in bytes.
 */
void smtc_modem_hal_context_store(const modem_context_type_t ctx_type, uint32_t offset, const uint8_t *buffer, const uint32_t size)
{
   int rc = 0;

   if (offset != 0) {
	   LOG_ERR("%d context store offset %u", ctx_type, offset);
   }
   if (InitSettingsSubsys())
   {
	  switch (ctx_type)
	  {
#if 0
		 case CONTEXT_MODEM:
			rc = settings_save_one(ADDR_FLASH_MODEM_CONTEXT, (void *) buffer, size);
			break;

		 case CONTEXT_LR1MAC:
			rc = settings_save_one(ADDR_FLASH_LR1MAC_CONTEXT, (void *) buffer, size);
			break;

		 case CONTEXT_DEVNONCE:
			rc = settings_save_one(ADDR_FLASH_DEVNONCE_CONTEXT, (void *) buffer, size);
			break;

		 case CONTEXT_SECURE_ELEMENT:
			rc = settings_save_one(ADDR_FLASH_SECURE_ELEMENT_CONTEXT, (void *) buffer, size);
			break;
#endif /* #if 0 */
		case CONTEXT_MODEM:
			rc = settings_save_one(ADDR_FLASH_MODEM_CONTEXT, (void *) buffer, size);
			break;
		case CONTEXT_KEY_MODEM:
			rc = settings_save_one(ADDR_FLASH_MODEM_KEY_CONTEXT, (void *) buffer, size);
			break;
		case CONTEXT_LORAWAN_STACK:
			rc = settings_save_one(ADDR_FLASH_LORAWAN_CONTEXT, (void *) buffer, size);
			break;
		case CONTEXT_FUOTA:
			rc = settings_save_one(ADDR_FLASH_FUOTA, (void *) buffer, size);
			break;
		case CONTEXT_SECURE_ELEMENT:
			rc = settings_save_one(ADDR_FLASH_SECURE_ELEMENT_CONTEXT, (void *) buffer, size);
			break;
		case CONTEXT_STORE_AND_FORWARD:
			rc = settings_save_one(ADDR_FLASH_STORE_AND_FORWARD, (void *) buffer, size);
			break;
			
		 default:
			LOG_ERR("Unknown context: %u", (uint32_t) ctx_type);
			k_sleep(K_MSEC(200));
			k_panic();
			break;
	  }

	  if (rc != 0)
	  {
		 LOG_ERR("Settings Set Context %u. Error=%d", (uint32_t) ctx_type, rc);
	  }
	  else
	  {
		 LOG_DBG("Context %u stored.", (uint32_t) ctx_type);
	  }
   }
}

/* ------------ Crashlog management ------------*/

/**
 * @brief Stores the crashlog.
 *
 * @remark This function is used to store the Modem crashlog in a non volatile memory.
 *
 * @param [in] crashlog   Buffer of 32 bytes containing crashlog data to store.
 */
void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length )
{
	crashlog_length_noinit = MIN( crash_string_length, CRASH_LOG_SIZE );
	memcpy( crashlog_buff_noinit, crash_string, crashlog_length_noinit );
	crashlog_available_noinit = true;
}

void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length )
{
	*crash_string_length = ( crashlog_length_noinit > CRASH_LOG_SIZE ) ? CRASH_LOG_SIZE : crashlog_length_noinit;
	memcpy( crash_string, crashlog_buff_noinit, *crash_string_length );
}

/**
 * @brief Get the previously stored crashlog status.
 *
 * @remark This function is used to get the Modem crashlog status from a
 * non volatile memory. This status will allow the Modem to handle crashlog
 * send task if needed after a crash.
 *
 * @return bool True if a crashlog is available, false otherwise.
 */
bool smtc_modem_hal_crashlog_get_status()
{
	return crashlog_available_noinit;
}

/* ------------ assert management ------------*/

/**
 * @brief Returns the source function and the source line number where
 *		the assert error has occurred.
 *
 * @param func
 * @param line
 */
void smtc_modem_hal_assert_fail(uint8_t *func, uint32_t line)
{
   smtc_modem_hal_crashlog_store(func, strlen(func));
   smtc_modem_hal_crashlog_set_status(true);
   smtc_modem_hal_print_trace(
	  "\x1B[0;31m"  // red color
	  "ERROR: Crash log :%s:%u\n"
	  "\x1B[0m",  // revert default color
	  func, line);
   smtc_modem_hal_reset_mcu();
}

/* ------------ Random management ------------*/

/**
 * @brief Returns a 32bits random number.
 *
 * @return uint32_t Generated random number.
 */
uint32_t smtc_modem_hal_get_random_nb(void)
{
   static bool srandInit = false;
   if (!srandInit)
   {
	  srand(k_uptime_ticks());
	  srandInit = true;
   }
   return (uint32_t) rand();
}

/**
 * @brief Returns an unsigned random number between min and max.
 *
 * @param [in] val_1 first range unsigned value.
 * @param [in] val_2 second range unsigned value.
 *
 * @return uint32_t Generated random unsigned number between smallest value
 *	 and biggest value (between val_1 and val_2).
 */
uint32_t smtc_modem_hal_get_random_nb_in_range(const uint32_t val_1, const uint32_t val_2)
{
	uint32_t rnd = smtc_modem_hal_get_random_nb();
	if (val_1 <= val_2)
	{
		uint32_t limit = val_2 - val_1 + 1;
		if (limit != 0)
			rnd = (rnd % limit) + val_1;
	}
	else
	{
		uint32_t limit = val_1 - val_2 + 1;
		if (limit != 0)
			rnd = (rnd % limit) + val_2;
	}
	return rnd;
}

/**
 * @brief Returns a signed random number between min and max.
 *
 * @param [in] val_1 first range signed value.
 * @param [in] val_2 second range signed value.
 *
 * @return int32_t Generated random signed number between smallest value
 *	 and biggest value (between val_1 and val_2).
 */
int32_t smtc_modem_hal_get_signed_random_nb_in_range(const int32_t val_1, const int32_t val_2)
{
   uint32_t tmp_range = 0;
   if (val_1 <= val_2)
   {
	  tmp_range = (val_2 - val_1);
	  return (int32_t) ((val_1 + smtc_modem_hal_get_random_nb_in_range(0, tmp_range)));
   }
   else
   {
	  tmp_range = (val_1 - val_2);
	  return (int32_t) ((val_2 + smtc_modem_hal_get_random_nb_in_range(0, tmp_range)));
   }
}

/* ------------ Radio env management ------------*/
#ifdef SMTC_MODEM_HAL_IRQ_FROM_SID_PAL
void smtc_modem_hal_radio_irq(void)
{
	app_event_lbm();
	k_work_submit(&halDio1WorkItem);
}
#else
static void radio_irq(uint32_t pin, void * callback_arg)
{
	app_event_lbm();
	// Offload the DIO1 handling to a work queue thread.
	k_work_submit(&halDio1WorkItem);
}
#endif /* !SMTC_MODEM_HAL_IRQ_FROM_SID_PAL */

/**
 * @brief Configure the radio DIO1 interrupt callback.
 *
 * @param [in] callback	 Callback that will be called in case of timer irq.
 * @param [in] context	  Context that will be passed on callback argument.
 */
void smtc_modem_hal_irq_config_radio_irq(void(*callback)(void *context), void *context)
{
#ifdef SMTC_MODEM_HAL_IRQ_FROM_SID_PAL
	HalDio1Callback = callback;
	halDio1Context = context;
#else
#ifdef LR11XX
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	if (sid_pal_gpio_set_irq(cfg->gpios.int1,
#else
	const radio_sx126x_device_config_t *cfg = get_radio_cfg();
	if (sid_pal_gpio_set_irq(cfg->gpio_int1,
#endif
		SID_PAL_GPIO_IRQ_TRIGGER_EDGE, radio_irq, NULL) != SID_ERROR_NONE) {
	  	LOG_ERR("Set DIO1 interrupt callback error");
	} else {
		HalDio1Callback = callback;
		halDio1Context = context;
	}
#endif /* !SMTC_MODEM_HAL_IRQ_FROM_SID_PAL */
}

/**
 * @brief Clear any MCU-layer pending radio IRQ flags.
 */
void smtc_modem_hal_radio_irq_clear_pending(void)
{
#if 0
   // Copied from gpio_nrfx.c.
   struct gpio_nrfx_cfg
   {
	  struct gpio_driver_config common;
	  NRF_GPIO_Type *port;
	  uint32_t edge_sense;
	  uint8_t port_num;
   };

   sx126x_hal_context_t const * const radioContext = modem_context_get_modem_radio_ctx();
   uint8_t ch;
   const struct gpio_nrfx_cfg *gpioCfg = (const struct gpio_nrfx_cfg *) (radioContext->gpioDio1.port->config);
   uint32_t abs_pin = NRF_GPIO_PIN_MAP(gpioCfg->port_num, radioContext->gpioDio1.pin);
   nrfx_err_t err = nrfx_gpiote_channel_get(abs_pin, &ch);
   if (err != NRFX_SUCCESS)
   {
	  LOG_ERR("GPIOTE channel error: %u", (uint32_t) err);
   }
   else
   {
	  LOG_DBG("Clear IRQ for GPIO Port=%u Pin=%u", gpioCfg->port_num, radioContext->gpioDio1.pin);

	  nrf_gpiote_event_clear(NRF_GPIOTE, nrf_gpiote_in_event_get(ch));
   }
#endif /* #if 0 */
}

/**
 * @brief Start radio TCXO.
 *
 * @remark In case used radio has no TCXO please implement an empty function.
 */
void smtc_modem_hal_start_radio_tcxo(void)
{
   // put here the code that will start the tcxo if needed
}

/**
 * @brief Stop radio TCXO.
 *
 * @remark In case used radio has no TCXO please implement an empty function.
 */
void smtc_modem_hal_stop_radio_tcxo(void)
{
   // put here the code that will stop the tcxo if needed
}

/**
 * @brief Get TCXO startup delay, in ms.
 *
 * @remark In case used radio has no TCXO please implement a function which returns 0.
 *
 * @return uint32_t TCXO startup delay in ms.
 */
uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms(void)
{
#ifdef LR11XX
	const radio_lr11xx_device_config_t *cfg = get_radio_cfg();
	if (cfg->tcxo_config.ctrl == LR11XX_TCXO_CTRL_NONE)
		return 0;
	else
		return 2;
#else
   return 0;
#endif
}

/* ------------ Environment management ------------*/

/**
 * @brief Return the battery level.
 *	A value between 0 (for 0%) and 255 (for 100%).
 *
 * @return uint8_t Battery level for lorawan stack.
 */
uint8_t smtc_modem_hal_get_battery_level(void)
{
   uint8_t bat = 254;
   LOG_DBG("Battery: %u", bat);
   return bat;
}

/**
 * @brief Return MCU temperature in celsius.
 *
 * @return int8_t MCU temperature in celsius.
 */
int8_t smtc_modem_hal_get_temperature(void)
{
   int8_t tempC;
   struct sensor_value temperature;

	temperature.val1 = 0;
	temperature.val2 = 0;

   TemperatureGet(&temperature);

   tempC = (int8_t) temperature.val1;

   // Round up?
   if (temperature.val2 >= 500000)
   {
	  tempC += 1;
   }

   LOG_DBG("Temperature: %d C", tempC);

   return tempC;
}

/**
 * @brief Return mcu voltage (can be needed for dm uplink payload).
 *	Indicates the current battery voltage.
 *
 * @return uint8_t MCU voltage, in units of 20mV.
 */
uint8_t smtc_modem_hal_get_voltage(void)
{
   uint16_t measure_vref_mv = 3300;

   LOG_DBG("Voltage: %u mV", measure_vref_mv);

   // convert voltage from mv to cloud readable (1/50V = 20mv)
   return (uint8_t) (measure_vref_mv / 20);
}

/**
 * @brief Return board wake up delay in ms.
 *
 * @return uint8_t Board wake up delay in ms.
 */
int8_t smtc_modem_hal_get_board_delay_ms(void)
{
   return 1;
}

/* ------------ Trace management ------------*/

/**
 * @brief Prints debug trace.
 *
 * @param variadics arguments.
 */
#define TMP_SIZE		128
void smtc_modem_hal_print_trace(const char *fmt, ...)
{
#ifdef CONFIG_LOG
	static uint8_t log__str_joined_len;
	static char log__str_joined[192];
	static bool subsequent = false;
	char str[128];
	uint8_t this_len;
	va_list args;
	va_start(args, fmt);

	vsnprintf(str, sizeof(str), fmt, args);
	va_end(args);

	this_len = strlen(str); // strlen excludes terminating null byte
	if (!subsequent) {
		if (str[this_len-1] == '\n' || str[0] == 0x1b) {
			/* new line terminated string, or ansi color string */
			LOG_PRINTK("%s", str);
		} else {
			/* first part of concated string */
			strncpy(log__str_joined, str, sizeof(log__str_joined));
			log__str_joined_len = this_len;
			subsequent = true;
		}
	} else {
		uint8_t remain = sizeof(log__str_joined) - log__str_joined_len;
		strncat(log__str_joined, str, remain);
		log__str_joined_len += this_len;
		if (str[this_len-1] == '\n') {
			/* termination of concated string */
			LOG_PRINTK("%s", log__str_joined);
			subsequent = false;
		}
	}
#endif /* CONFIG_LOG */
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief Initializes the Settings subsystem, if necessary.
 *
 * @return True if Settings subsystem is initialized.
 */
static bool InitSettingsSubsys(void)
{

   if (!settingsSubsysInit)
   {
	  int rc = settings_subsys_init();
	  if (rc)
	  {
		 LOG_ERR("Settings subsys initialization: %d", rc);
		 k_sleep(K_MSEC(200));
		 k_panic();
	  }
	  else
	  {
		 settingsSubsysInit = true;
	  }
   }
   return settingsSubsysInit;
}

static int DirectLoadHandler(const char *name, size_t len, settings_read_cb read_cb,
							 void *cb_arg, void *param)
{
   const char *next;
   size_t name_len;
   int rc;
   DirectLoadParams_t *directLoadParams = (DirectLoadParams_t *) param;

   name_len = settings_name_next(name, &next);

   if (name_len == 0)
   {
	  if (len == directLoadParams->len)
	  {
		 rc = read_cb(cb_arg, directLoadParams->dest, len);
		 if (rc >= 0)
		 {
			directLoadParams->fetched = true;
			return 0;
		 }

		 LOG_ERR("Read CB error: %d", rc);
		 return rc;
	  }
	  return -EINVAL;
   }

   // Other keys aren't served by the callback.
   // Return success in order to skip them and keep storage processing.
   return 0;
}

/**
 * @brief DIO1 expiration work queue handler.
 *
 * @param [in] work Work object.
 */
static void HalDio1WorkHandler(struct k_work *work)
{
	if (HalDio1Callback != NULL)
	{
		LOG_DBG("DIO1 interrupt. Call handler.");
		HalDio1Callback(halDio1Context);
	} else {
		LOG_WRN("DIO1 interrupt. no handler.");
	}

#ifdef SMTC_MODEM_HAL_IRQ_FROM_SID_PAL
	halo_drv_semtech_ctx_t* drv = lr11xx_get_drv_ctx();
	if (drv->post_scan_recovery) {
		drv->post_scan_recovery = false;
		lr11xx_restore_transceiver();
	}
#endif /* SMTC_MODEM_HAL_IRQ_FROM_SID_PAL */
}

/**
 * @brief Timer expiration work queue handler.
 *
 * @param [in] work Work object.
 */
static void HalTimerWorkHandler(struct k_work *work)
{
   if (HalTimerCallback != NULL)
   {
	  LOG_DBG("Timeout. Call handler.");
	  HalTimerCallback(halTimerContext);
   }
}

/**
 * @brief Timer expiration handling function.
 *
 * @param [in] timer Timer object.
 */
static void HalTimerHandler(struct k_timer *timer)
{
	smtc_modem_hal_timer_running = false;
	if (halTimerIrqEnabled)
	{
		app_event_lbm();
		if (timer != &halTimer)
		{
			LOG_ERR("Unexpected timer: %p", timer);
		}
		else if (HalTimerCallback != NULL)
		{
			// Offload the timer handling to a work queue thread.
			k_work_submit(&halTimerWorkItem);
		}
	} else
		halTimerIrqPending = true;
}

#define DEFAULT_TEMPERATURE			25

static const struct device *const device = DEVICE_DT_GET(DT_NODELABEL(temp));

static void TemperatureGet(struct sensor_value *temperature)
{
   struct sensor_value val = {DEFAULT_TEMPERATURE, 0};

   int err = sensor_sample_fetch(device);
   if (!err)
   {
	  // Get the device die temperature in degrees Celsius.
	  err = sensor_channel_get(device, SENSOR_CHAN_DIE_TEMP, &val);
	  if (!err)
	  {
		 *temperature = val;

		 LOG_INF("Device temperature: %d.%06d C", val.val1, val.val2);
	  }
   }

}

void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... )
{
	uint8_t out_buff[255] = { 0 };
	uint8_t out_len	   = snprintf( ( char* ) out_buff, sizeof( out_buff ), "%s:%u ", func, line );

	va_list args;
	va_start( args, fmt );
	out_len += vsprintf( ( char* ) &out_buff[out_len], fmt, args );
	va_end( args );

	smtc_modem_hal_crashlog_store( out_buff, out_len );

	LOG_ERR( "Modem panic: %s\n", out_buff );
	smtc_modem_hal_reset_mcu( );
}

void smtc_modem_hal_set_ant_switch( bool is_tx_on )
{
	//TODO ? does radio control antenna switch or host mcu ?
}

void smtc_modem_hal_user_lbm_irq( void )
{
	// TODO -- Do nothing in case implementation is bare metal
}

void smtc_modem_hal_crashlog_set_status( bool available )
{
	crashlog_available_noinit = available;
}

uint16_t smtc_modem_hal_get_voltage_mv( void )
{
	return 3300;
}


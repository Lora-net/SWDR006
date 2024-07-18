/**************************************************************************//**
* COPYRIGHT 2022 CONNECTED DEVELOPMENT, A DIVISION OF EXPONENTIAL
* TECHNOLOGY GROUP.
*
* ALL RIGHTS RESERVED BY AND FOR THE EXCLUSIVE BENEFIT OF
* CONNECTED DEVELOPMENT, A DIVISION OF EXPONENTIAL TECHNOLOGY GROUP.
*
* CONNECTED DEVELOPMENT, A DIVISION OF EXPONENTIAL TECHNOLOGY GROUP -
* CONFIDENTIAL AND PROPRIETARY INFORMATION AND/OR TRADE SECRET.
*
* NOTICE: ALL CODE, PROGRAM, INFORMATION, SCRIPT, INSTRUCTION,
* DATA, AND COMMENT HEREIN IS AND SHALL REMAIN THE CONFIDENTIAL
* INFORMATION AND PROPERTY OF CONNECTED DEVELOPMENT, A DIVISION OF
* EXPONENTIAL TECHNOLOGY GROUP.
* USE AND DISCLOSURE THEREOF, EXCEPT AS STRICTLY AUTHORIZED IN A
* WRITTEN AGREEMENT SIGNED BY CONNECTED DEVELOPMENT, A DIVISION OF
* EXPONENTIAL TECHNOLOGY GROUP, IS PROHIBITED.
*******************************************************************************
* @file
* @brief  Connected Development implementation of the SX126x radio HAL context.
* @details  Copied from the example in the SDK at:
*			 SWSD001\shields\SX126X\radio_drivers_hal
******************************************************************************/

/*!
 * \file	  sx126x_hal_context.h
 *
 * \brief	 Declaration of SX126X HAL context
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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
#ifndef SX126X_HAL_CONTEXT_H
#define SX126X_HAL_CONTEXT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

// The context contains the SPI device and GPIOs to the SX126X.
typedef struct
{
#if 0
   struct spi_dt_spec	  spiSpec;	// SPI bus device
   struct gpio_dt_spec	 gpioCs;	 // NSS Slave Select GPIO
   struct gpio_dt_spec	 gpioReset;  // Reset GPIO
   struct gpio_dt_spec	 gpioBusy;   // Busy GPIO
   struct gpio_dt_spec	 gpioDio1;   // DIO1 GPIO
#endif  /* #if 0 */
   //char foo;
   const void *halo_ctx;
} sx126x_hal_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // SX126X_HAL_CONTEXT_H

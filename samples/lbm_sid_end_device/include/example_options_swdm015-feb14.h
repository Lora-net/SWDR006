/**
 * @file	  example_options.h
 *
 * @brief	 User options to be used in example applications
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

#ifndef EXAMPLE_OPTIONS_H
#define EXAMPLE_OPTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "smtc_modem_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief LoRaWAN User credentials
 * joineu 97742779212C2BB6
 * deveui 3BBE1562BCD35D32
 *	key 195029CB37A463742D063AF660281139
 */
#define USER_LORAWAN_DEVICE_EUI						\
	{												  \
		0x3B, 0xBE, 0x15, 0x62, 0xBC, 0xD3, 0x5D, 0x32 \
	}
#define USER_LORAWAN_JOIN_EUI						  \
	{												  \
		0x97, 0x74, 0x27, 0x79, 0x21, 0x2C, 0x2B, 0xB6 \
	}
#define USER_LORAWAN_GEN_APP_KEY																	   \
	{																								  \
		0x19, 0x50, 0x29, 0xCB, 0x37, 0xA4, 0x63, 0x74, 0x2D, 0x06, 0x3A, 0xF6, 0x60, 0x28, 0x11, 0x39 \
	}
#define USER_LORAWAN_APP_KEY																		   \
	{																								  \
		0x19, 0x50, 0x29, 0xCB, 0x37, 0xA4, 0x63, 0x74, 0x2D, 0x06, 0x3A, 0xF6, 0x60, 0x28, 0x11, 0x39 \
	}

/**
 * @brief Modem Region define
 */
#ifndef MODEM_EXAMPLE_REGION
#if !defined( SX128X )
#define MODEM_EXAMPLE_REGION SMTC_MODEM_REGION_EU_868
#else
#define MODEM_EXAMPLE_REGION SMTC_MODEM_REGION_WW2G4
#endif
#endif  // MODEM_EXAMPLE_REGION
// clang-format on

#ifdef __cplusplus
}
#endif

#endif  // EXAMPLE_OPTIONS_H

/* --- EOF ------------------------------------------------------------------ */

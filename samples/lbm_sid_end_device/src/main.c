/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <app.h>
#include <zephyr/logging/log.h>
#include <sidewalk_version.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_SIDEWALK_LOG_LEVEL);

int main(void)
{
	PRINT_SIDEWALK_VERSION();

	app_start();

	return 0;
}

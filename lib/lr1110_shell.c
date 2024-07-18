#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "zephyr/sys/util.h"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include "halo_lr11xx_radio.h"
#include "lr1110_shell.h"

LOG_MODULE_REGISTER(lr1110_cli, CONFIG_SIDEWALK_LOG_LEVEL);

#define NB_MAX_SV		8

#define LR1110_STACK_SIZE 5120
#define LR1110_WORKER_PRIO 5

struct k_work_q lr1110_work_q;
K_THREAD_STACK_DEFINE(lr1110_work_q_stack, LR1110_STACK_SIZE);

struct gnss_work_t {
	struct k_work work;
	const struct shell *shell;
	bool nRead_write;
	int value;
	float lon, lat;
};

struct gnss_work_t gnss_work;
static struct sid_handle *sidewalk_handle;
void app_event_send_wake(void); // implemented in application
void app_event_wifi_scan(void); // implemented in application

extern lr11xx_gnss_solver_assistance_position_t assistance_position; // application declared

int gnss_scan_timer_set(unsigned);	// implement in application
unsigned gnss_scan_timer_get(void);

uint32_t CLI_sleep_to_full_power_us = 500;
uint32_t CLI_tcxo_us;

extern wifi_configuration_scan_t wifi_configuration;

void cmd_print_lr11xx_version(const struct shell *shell, size_t argc, char **argv)
{
	shell_info(shell, "TODO lr1110 get version");
}

static void cmd_gnss_autonomus_work(struct k_work *item)
{
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	void* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}
	lr11xx_status_t status = lr11xx_gnss_scan_autonomous(drv_ctx,
		0,
		LR11XX_GNSS_OPTION_BEST_EFFORT,
		LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK | LR11XX_GNSS_RESULTS_DOPPLER_MASK | LR11XX_GNSS_RESULTS_BIT_CHANGE_MASK,
		NB_MAX_SV
	);
	if (status == LR11XX_STATUS_ERROR)
		shell_error(gnss_work->shell, "autonomus scan fail");
	else
		shell_info(gnss_work->shell, "autonomus scan started");
}

static void cmd_gnss_assisted_work(struct k_work *item)
{
	struct sid_timespec curr_time;
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	void* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}

	sid_error_t ret = sid_get_time(sidewalk_handle, SID_GET_GPS_TIME, &curr_time);
	if (SID_ERROR_NONE != ret) {
		shell_error(gnss_work->shell, "sid_get_time fail %d", ret);
		return;
	}
	lr11xx_status_t status;
	status = lr11xx_gnss_set_assistance_position(drv_ctx, &assistance_position);
	if (status == LR11XX_STATUS_ERROR) {
		shell_error(gnss_work->shell, "set AP fail");
		return;
	}
	status = lr11xx_gnss_scan_assisted(drv_ctx,
		curr_time.tv_sec,
		LR11XX_GNSS_OPTION_BEST_EFFORT,
		LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK | LR11XX_GNSS_RESULTS_DOPPLER_MASK | LR11XX_GNSS_RESULTS_BIT_CHANGE_MASK,
		NB_MAX_SV
	);
	if (status == LR11XX_STATUS_ERROR)
		shell_error(gnss_work->shell, "assisted scan fail");
	else
		shell_info(gnss_work->shell, "assisted scan started %u, %f %f", curr_time.tv_sec, assistance_position.latitude, assistance_position.longitude);
}

static void cmd_getstatus_work(struct k_work *item)
{
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	halo_drv_semtech_ctx_t* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}
	lr11xx_status_t status = lr11xx_system_clear_reset_status_info(drv_ctx);
	if (status == LR11XX_STATUS_ERROR)
		shell_error(gnss_work->shell, "getstatus fail");
	else {
		shell_info(gnss_work->shell, "stat1:%x stat2:%x", drv_ctx->last.stat1, drv_ctx->last.stat2);
	}
}

static int cmd_lr1110_gnss_autonomus(const struct shell *shell, size_t argc, char **argv)
{
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute gnss command, previous gnss has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_gnss_autonomus_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}

static int cmd_lr1110_gnss_assisted(const struct shell *shell, size_t argc, char **argv)
{
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute gnss command, previous gnss has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_gnss_assisted_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}

static void cmd_gnss_scan_mode_work(struct k_work *item)
{
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	void* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}

	if (gnss_work->nRead_write) {
		lr11xx_status_t status = lr11xx_gnss_set_scan_mode(drv_ctx, gnss_work->value);
		if (status == LR11XX_STATUS_ERROR)
			shell_error(gnss_work->shell, "gnssSetScan fail");
		else
			shell_info(gnss_work->shell, "gnssSetScan ok");
	} else
		LOG_WRN("provide value");
	/* lr11xx_gnss_scan_mode_t : 0 is single legacy scan, 3 is multiscan (5 fast scans)
	 */
}

static void cmd_gnss_context_work(struct k_work *item)
{
	lr11xx_gnss_context_status_bytestream_t gnss_context;
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	void* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}
	lr11xx_status_t status = lr11xx_gnss_get_context_status(drv_ctx, gnss_context);
	if (status == LR11XX_STATUS_ERROR)
		shell_error(gnss_work->shell, "fail");
	else {
		shell_hexdump(gnss_work->shell, gnss_context, LR11XX_GNSS_CONTEXT_STATUS_LENGTH);
	}
}

static void cmd_gnss_almanac_age_work(struct k_work *item)
{
	struct sid_timespec curr_time;
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	void* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}
	uint16_t almanac_age;
	sid_error_t ret = sid_get_time(sidewalk_handle, SID_GET_GPS_TIME, &curr_time);
	if (SID_ERROR_NONE != ret) {
		shell_error(gnss_work->shell, "sid_get_time fail %d", ret);
		return;
	}
	lr11xx_status_t status = lr11xx_gnss_get_almanac_age_for_satellite(drv_ctx, gnss_work->value, &almanac_age);
	if (status == LR11XX_STATUS_ERROR)
		shell_error(gnss_work->shell, "fail");
	else {
		const uint16_t days_between_2019_rollover_to_epoch = 14336;
		const uint16_t almanac_age_days_since_gps_epoch    = almanac_age + days_between_2019_rollover_to_epoch;
		const uint16_t now_days = curr_time.tv_sec / 3600 / 24;
		shell_info(gnss_work->shell, "almanac age from chip %u, for SV %d", almanac_age, gnss_work->value);
		shell_info(gnss_work->shell, "almanac age %u days", now_days - almanac_age_days_since_gps_epoch);
	}
}

static void cmd_gnss_ap_work(struct k_work *item)
{
	lr11xx_status_t status;
	lr11xx_gnss_solver_assistance_position_t ap;
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	void* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}

	if (gnss_work->nRead_write) {
		ap.latitude = gnss_work->lat;
		ap.longitude = gnss_work->lon;
		status = lr11xx_gnss_set_assistance_position(drv_ctx, &ap);
		if (status == LR11XX_STATUS_ERROR)
			shell_error(gnss_work->shell, "set AP fail");
		else {
			shell_error(gnss_work->shell, "set AP ok %f, %f", ap.latitude, ap.longitude);
			assistance_position.latitude = ap.latitude;
			assistance_position.longitude = ap.longitude;
		}

	} else {
		status = lr11xx_gnss_read_assistance_position(drv_ctx, &ap);
		if (status == LR11XX_STATUS_ERROR)
			shell_error(gnss_work->shell, "get AP fail");
		else
			shell_error(gnss_work->shell, "get AP ok %f, %f", ap.latitude, ap.longitude);
	}
}

static void cmd_gnssconst_work(struct k_work *item)
{
	unsigned char readout;
	struct gnss_work_t *gnss_work = CONTAINER_OF(item, struct gnss_work_t, work);
	void* drv_ctx = lr11xx_get_drv_ctx();
	if (lr11xx_system_wakeup(drv_ctx) != LR11XX_STATUS_OK) {
		shell_error(gnss_work->shell, "wake-up fail");
		return;
	}
/*    LR1110_GNSS_GPS_MASK    = 0x01,
    LR1110_GNSS_BEIDOU_MASK = 0x02,*/
	
	lr11xx_status_t status;
	if (gnss_work->nRead_write)
		status = lr11xx_gnss_set_constellations_to_use(drv_ctx, gnss_work->value);
	else
		status = lr11xx_gnss_read_used_constellations(drv_ctx, &readout);
	if (status == LR11XX_STATUS_ERROR)
		shell_error(gnss_work->shell, "gnssconst fail");
	else {
		if (gnss_work->nRead_write)
			shell_info(gnss_work->shell, "gnssconst ok");
		else
			shell_info(gnss_work->shell, "gnssconst ok, read back %d", readout);
	}
}

static int cmd_lr1110_get_status(const struct shell *shell, size_t argc, char **argv)
{
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute getstatus command, previous has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_getstatus_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}

static int cmd_lr1110_gnss_constellation(const struct shell *shell, size_t argc, char **argv)
{
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute gnss command, previous has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

	LOG_INF("argc:%d", argc);
	if (argc == 2) {
		gnss_work.nRead_write = true;
		sscanf(argv[1], "%d", &gnss_work.value);
	} else
		gnss_work.nRead_write = false;

	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_gnssconst_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}

static int cmd_lr1110_gnss_scan_mode(const struct shell *shell, size_t argc, char **argv)
{
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute gnss command, previous has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

	if (argc == 2) {
		gnss_work.nRead_write = true;
		sscanf(argv[1], "%d", &gnss_work.value);
	} else
		gnss_work.nRead_write = false;

	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_gnss_scan_mode_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}

static int cmd_lr1110_assistance_position(const struct shell *shell, size_t argc, char **argv)
{
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute gnss command, previous has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

	if (argc == 3) {
		gnss_work.nRead_write = true;
		sscanf(argv[1], "%f", &gnss_work.lat);
		sscanf(argv[2], "%f", &gnss_work.lon);
		shell_info(shell, "argv %f, %f", gnss_work.lat, gnss_work.lon);
	} else
		gnss_work.nRead_write = false;

	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_gnss_ap_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}


static int cmd_lr1110_get_gnss_context(const struct shell *shell, size_t argc, char **argv)
{
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute gnss command, previous has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_gnss_context_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}

static int cmd_lr1110_wifi_scan_type(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 2) {
        if (argv[1][0] == 'b')
            wifi_configuration.signal_type = LR11XX_WIFI_TYPE_SCAN_B;
        else if (argv[1][0] == 'g')
            wifi_configuration.signal_type = LR11XX_WIFI_TYPE_SCAN_G;
        else if (argv[1][0] == 'n')
            wifi_configuration.signal_type = LR11XX_WIFI_TYPE_SCAN_N;
        else if (argv[1][0] == 'a')
            wifi_configuration.signal_type = LR11XX_WIFI_TYPE_SCAN_B_G_N;
        else {
            shell_error(shell, "provide b, g, n or a");
            return EINVAL;
        }
    }

    switch (wifi_configuration.signal_type) {
        case LR11XX_WIFI_TYPE_SCAN_B:
            shell_info(shell, "B");
            break;
        case LR11XX_WIFI_TYPE_SCAN_G:
            shell_info(shell, "G");
            break;
        case LR11XX_WIFI_TYPE_SCAN_N:
            shell_info(shell, "N");
            break;
        case LR11XX_WIFI_TYPE_SCAN_B_G_N:
            shell_info(shell, "B_G_N");
            break;
        default:
            shell_info(shell, "? %d ?", wifi_configuration.signal_type);
            break;
    }
    return 0;
}

static int cmd_lr1110_wifi_tps(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 2) {
        unsigned foo;
	    sscanf(argv[1], "%u", &foo);
        wifi_configuration.timeout_per_scan = foo;
    }
    shell_info(shell, "timeout_per_scan: %u", wifi_configuration.timeout_per_scan);
    return 0;
}

static int cmd_lr1110_wifi_spc(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 2) {
        unsigned foo;
	    sscanf(argv[1], "%u", &foo);
        wifi_configuration.nb_scan_per_channel = foo;
    }
    shell_info(shell, "nb_scan_per_channel: %u", wifi_configuration.nb_scan_per_channel);
    return 0;
}

static int cmd_lr1110_wifi_max_result(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 2) {
        unsigned foo;
	    sscanf(argv[1], "%u", &foo);
        wifi_configuration.base.max_result = foo;
    }
    shell_info(shell, "max_result: %u", wifi_configuration.base.max_result);
    return 0;
}

static int cmd_lr1110_wifi_scan(const struct shell *shell, size_t argc, char **argv)
{
    app_event_wifi_scan();
	return 0;
}

static int cmd_lr1110_get_almanac_age(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "provide SV number");
		return ENOEXEC;
	}
	if (k_work_busy_get(&gnss_work.work) != 0) {
		shell_error(shell,
			    "Can not execute gnss command, previous has not completed yet");
		return ENOEXEC;	// or EINVAL if invalid
	}

    sscanf(argv[1], "%d", &gnss_work.value);
	gnss_work.shell = shell;
	k_work_init(&gnss_work.work, cmd_gnss_almanac_age_work);
	k_work_submit_to_queue(&lr1110_work_q, &gnss_work.work);
	return 0;
}

static int cmd_lr1110_tt(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 2) {
	    sscanf(argv[1], "%u", &CLI_tcxo_us);
    }
    shell_info(shell, "tcxo_delay_us: %u", CLI_tcxo_us);
    return 0;
}

static int cmd_lr1110_sfp(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 2) {
	    sscanf(argv[1], "%u", &CLI_sleep_to_full_power_us);
    }
    shell_info(shell, "SFP: %u", CLI_sleep_to_full_power_us);
    return 0;
}

static int cmd_lr1110_wake(const struct shell *shell, size_t argc, char **argv)
{
    app_event_send_wake();
    return 0;
}

static int cmd_lr1110_scan_timer(const struct shell *shell, size_t argc, char **argv)
{
	unsigned sec;
	if (argc == 1) {
		shell_info(shell, "scan timer %u seconds", gnss_scan_timer_get());
        return 0;
	} else if (argc == 2) {
        sscanf(argv[1], "%u", &sec);
        if (gnss_scan_timer_set(sec) != 0) {
            shell_error(shell, "gnss_scan_timer_set() failed");
            return ENOEXEC;
        }
        return 0;
    } else return EINVAL;
}


SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_services,
	SHELL_CMD_ARG(version, NULL, "print version LR1110", cmd_print_lr11xx_version, 1, 1),
	SHELL_CMD_ARG(gaut, NULL, "gnss scan autonomus", cmd_lr1110_gnss_autonomus, 1, 1),
	SHELL_CMD_ARG(gass, NULL, "gnss scan assisted", cmd_lr1110_gnss_assisted, 1, 1),
	SHELL_CMD_ARG(status, NULL, "read status", cmd_lr1110_get_status, 1, 1),
	SHELL_CMD_ARG(gnssconst, NULL, "set gnss constellation, 1=gps, 2=beidou, 3=both", cmd_lr1110_gnss_constellation, 1, 1),
	SHELL_CMD_ARG(gnssmode, NULL, "set gnss scan mode, 0=legacy, 3=advanced", cmd_lr1110_gnss_scan_mode, 1, 1),
	SHELL_CMD_ARG(ap, NULL, "get/set assistance position <lat> <lon>", cmd_lr1110_assistance_position, 1, 3),
	SHELL_CMD_ARG(context, NULL, "get gnss context", cmd_lr1110_get_gnss_context, 1, 1),
	SHELL_CMD_ARG(age, NULL, "get almanac age", cmd_lr1110_get_almanac_age, 2, 2),
	SHELL_CMD_ARG(gt, NULL, "start-stop gnss auto-timer", cmd_lr1110_scan_timer, 1, 2),
	SHELL_CMD_ARG(sfp, NULL, "get/set sleep_to_full_power_us", cmd_lr1110_sfp, 1, 2),
	SHELL_CMD_ARG(tt, NULL, "get/set txco_delay_us to MAC", cmd_lr1110_tt, 1, 2),
	SHELL_CMD_ARG(wake, NULL, "call LR1110 wakeup (for sleep testing)", cmd_lr1110_wake, 1, 1),
	SHELL_SUBCMD_SET_END);

// command, subcommands, help, handler
SHELL_CMD_REGISTER(lr1110, &sub_services, "LR1110 testing CLI", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_services_wifi,
	SHELL_CMD_ARG(scan, NULL, "start wifi scan", cmd_lr1110_wifi_scan, 1, 1),
	SHELL_CMD_ARG(tps, NULL, "get-set timeout_per_scan", cmd_lr1110_wifi_tps, 1, 2),
	SHELL_CMD_ARG(spc, NULL, "get-set nb_scan_per_channel ", cmd_lr1110_wifi_spc, 1, 2),
	SHELL_CMD_ARG(max, NULL, "get-set max_result", cmd_lr1110_wifi_max_result, 1, 2),
	SHELL_CMD_ARG(type, NULL, "get-set scan type (b/g/n/all) g and n do the same", cmd_lr1110_wifi_scan_type, 1, 2),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(wifi, &sub_services_wifi, "LR1110 wifi CLI", NULL);

void LR1110_CLI_init(struct sid_handle *handle)
{
	sidewalk_handle = handle;

	k_work_queue_init(&lr1110_work_q);

	k_work_queue_start(&lr1110_work_q, lr1110_work_q_stack,
			   K_THREAD_STACK_SIZEOF(lr1110_work_q_stack), LR1110_WORKER_PRIO, NULL);
}

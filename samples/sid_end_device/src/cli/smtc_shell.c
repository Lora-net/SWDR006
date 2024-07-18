#include <stdio.h>
#include <zephyr/shell/shell.h>
#include <halo_lr11xx_radio.h>
#include <sidewalk.h>

volatile int csuso;

static int cmd_smtc_csuso(const struct shell *sh, size_t argc, char **argv)
{
	halo_drv_semtech_ctx_t *radio_ctx = lr11xx_get_drv_ctx();
	if (argc == 2) { // 2: value provided
		sscanf(argv[1], "%d", &csuso);
		shell_print(sh, "argv[1] is %s, %d", argv[1], csuso);
	} else {
		// no value given, read whats there
		shell_print(sh, "csuso %d", radio_ctx->suppress_rx_timeout.csuso);
	}
    return 0;
}

static int cmd_dfu(const struct shell *sh, size_t argc, char **argv)
{
	return sidewalk_event_send(SID_EVENT_NORDIC_DFU, NULL);
}

static int cmd_demo_board(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, CONFIG_BOARD);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_smtc,
    SHELL_CMD(board, NULL, "Show board name command.", cmd_demo_board),
    SHELL_CMD(dfu, NULL, "send NORDIC_DFU event", cmd_dfu),
	SHELL_CMD_ARG(csuso, NULL, "get/set carrier-sense microsecond offset", cmd_smtc_csuso, 0, 1),
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(smtc, &sub_smtc, "semtech commands", NULL);


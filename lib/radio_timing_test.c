#include <stdint.h>
#include <radio_timing_test.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(timing, CONFIG_SIDEWALK_LOG_LEVEL);
const struct device *uartDev = DEVICE_DT_GET(DT_NODELABEL(arduino_serial));

volatile static bool busy;
#define BUF_SIZE		8	// pending buffer
char buf[BUF_SIZE+1];	// pending buffer
uint8_t buf_len;	// pending buffer length

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
		case UART_TX_DONE:
			busy = false;
			if (buf_len != 0)
				uart_send_str(buf, buf_len);
			break;
		case UART_TX_ABORTED: break;
		case UART_RX_RDY: break;
		case UART_RX_BUF_REQUEST: break;
		case UART_RX_BUF_RELEASED: break;
		case UART_RX_DISABLED: break;
		case UART_RX_STOPPED: break;
	}
}

int radio_timing_test_init()
{
	int ret;
	if (!device_is_ready(uartDev)) {
		LOG_ERR("UART device not found!");
		return -1;
	}

	ret = uart_callback_set(uartDev, uart_callback, NULL);
	if (ret != 0) {
		if (ret == -ENOSYS)
			LOG_ERR("not supported = uart_callback_set()");
		else if (ret == -ENOTSUP)
			LOG_ERR("API not available = uart_callback_set()");
		else
			LOG_ERR("%d = uart_callback_set()", ret);
	}
	return ret;
}

int uart_send_str(const char *str, uint8_t len)
{
	if (busy) {
		strncpy(buf, str, BUF_SIZE);
		if (len >= BUF_SIZE)
			len = BUF_SIZE;
		buf_len = len;
		buf[buf_len] = 0;
		return len;
	}
	int ret = uart_tx(uartDev, str, len, SYS_FOREVER_US);
	if (ret != 0) {
		if (ret == -ENOTSUP)
			LOG_ERR("uart API not enabled");
		else if (ret == -EBUSY)
			LOG_ERR("already sending");
		else
			LOG_ERR("%d = uart_tx", ret);
	} else {
		buf_len = 0;	// nothing pending
		busy = true;
	}

	return ret;
}

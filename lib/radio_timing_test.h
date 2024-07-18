
#ifdef TIMING_TEST
	int radio_timing_test_init(void);
	int uart_send_str(const char *str, uint8_t len);
	#define TIMING_TEST_INIT		radio_timing_test_init()
	#define TIMING_TEST_SEND_STR(x, y)	uart_send_str(x, y)
#else
	#define TIMING_TEST_INIT
	#define TIMING_TEST_SEND_STR(x, y)
#endif

#ifndef COMPONENTS_SCANNER_SCANNER_UART
#define COMPONENTS_SCANNER_SCANNER_UART

#include "scanner_app.h"

#define UART_BAUDRATE   115200
#define UART_NUM        UART_NUM_1
#define TX_PIN          27
#define RX_PIN          14

scanner_status_t scanner_uart_init(void);

void scanner_uart_deinit(void);

void scanner_uart_write_bytes(const char *bytes, size_t len);

size_t scanner_uart_read_bytes(char *buffer, size_t len);

size_t scanner_uart_get_rx_lenght(void);

#endif /* COMPONENTS_SCANNER_SCANNER_UART */


#include "esp_log.h"
#include "driver/uart.h"

#include "scanner_app.h"
#include "scanner_uart.h"

#define TAG                     "scanner-uart"

scanner_status_t scanner_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    if(uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "install uart driver failed.");
        return SCANNER_STATUS_ERR;
    }

    if(uart_param_config(UART_NUM, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "config uart error!");
        return SCANNER_STATUS_ERR;
    }

    if(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "config uart pin failed!");
        return SCANNER_STATUS_ERR;
    }

    return SCANNER_STATUS_OK;
}

void scanner_uart_deinit(void) 
{
    uart_driver_delete(UART_NUM);
}

void scanner_uart_write_bytes(const char *bytes, size_t len) 
{
    uart_write_bytes(UART_NUM, bytes, len);
}

size_t scanner_uart_read_bytes(char *buffer, size_t len)
{
    return uart_read_bytes(UART_NUM, (uint8_t *)buffer, len, 50);
}

size_t scanner_uart_get_rx_lenght(void)
{
    size_t rx_length = 0;
    uart_get_buffered_data_len(UART_NUM, &rx_length);

    return rx_length;
}

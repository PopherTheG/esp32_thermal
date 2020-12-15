#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hw1258.h"
#include "scanner_uart.h"
#include "scanner_app.h"

#define TAG                     "hw1258"
#define DELAY_TIME              500
#define TIMEOUT                 1000
#define delay_ms(__ms)          vTaskDelay(pdMS_TO_TICKS(__ms))

scanner_status_t hw1258_init(void) {
    scanner_uart_write_bytes(QRCENA, strlen(QRCENA));
    delay_ms(10);
    hw1258_get_expected_response();
    
    scanner_uart_write_bytes(SCMMDT, strlen(SCMMDT));
    delay_ms(10);
    hw1258_get_expected_response();
    
    scanner_uart_write_bytes(MDTMIT, strlen(MDTMIT));
    delay_ms(10);
    hw1258_get_expected_response();
    
    scanner_uart_write_bytes(MDTSTA, strlen(MDTSTA));   
    delay_ms(10);
    hw1258_get_expected_response();

    scanner_uart_write_bytes(MDTGUN33, strlen(MDTGUN33));
    delay_ms(10);
    hw1258_get_expected_response();

    scanner_uart_write_bytes(MDTEXT1, strlen(MDTEXT1));   
    delay_ms(10);
    hw1258_get_expected_response();

    scanner_uart_write_bytes(CNTALW0, strlen(CNTALW0));
    delay_ms(10);
    hw1258_get_expected_response();

     scanner_uart_write_bytes(BREENA0, strlen(BREENA0));
    delay_ms(10);
    hw1258_get_expected_response();
    

    return SCANNER_STATUS_OK;
}

scanner_status_t hw1258_get_expected_response() {
    uint8_t response = 0;
    size_t len = scanner_uart_read_bytes((char *)&response, 1);
    if(len > 0) {
        if (response == ACK) {
            // ESP_LOGE(TAG, "%d", response);
            return SCANNER_STATUS_OK;
        }
    }

    return SCANNER_STATUS_ERR;
}
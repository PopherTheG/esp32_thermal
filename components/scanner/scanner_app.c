#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "regex.h"

#include "scanner_uart.h"
#include "scanner_app.h"
#include "hw1258.h"

#define TAG                 "scanner_app"
#define MUTEX_TIMEOUT_MS    50
#define MUTEX_LOCK()        xSemaphoreTake(scanner_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))
#define MUTEX_UNLOCK()      xSemaphoreGive(scanner_mutex)

static SemaphoreHandle_t scanner_mutex;
static char buffer[37];
static char cur_uuid[37];
static scanner_app_cb app_cb;
static uint8_t run_flag = 0;

regex_t regex;

static void scanner_app_task(void *pData)
{
    ESP_LOGI(TAG, "Start scanner app task!");
    int res;
    while (run_flag)
    {
        size_t len = scanner_uart_read_bytes(buffer, sizeof(buffer));
        if (len > 0)
        {
            MUTEX_LOCK();
            // ESP_LOGI(TAG, "len %d", len);
            // ESP_LOGI(TAG, "%.*s", len, buffer);
            res = regcomp(&regex, "^[0-9a-f]{8}-[0-9a-f]{4}-[0-5][0-9a-f]{3}-[089ab][0-9a-f]{3}-[0-9a-f]{12}", 1);
            if (res)
            {
                ESP_LOGE(TAG, "Could not compile regex");
                scanner_event_t event = {
                    .id = SCANNER_EVT_FAIL
                };
                
                app_cb(&event);
            }
            res = regexec(&regex, buffer, 0, NULL, 0);

            if (!res)
            {
                scanner_event_t event = {
                    .id = SCANNER_EVT_UUID_VALID
                };
                strcpy(cur_uuid, buffer);
                app_cb(&event);
            }
            else if (res == 1)
            {
                // ESP_LOGE(TAG, "invalid uuid");
                scanner_event_t event = {
                    .id = SCANNER_EVT_UUID_INVALID
                };                
                app_cb(&event);
            }
            else
            {
                regerror(res, &regex, buffer, len);

                scanner_event_t event = {
                    .id = SCANNER_EVT_FAIL
                };
                
                app_cb(&event);
                // ESP_LOGE(TAG, "Regex match failed: %s", buffer);
            }

            regfree(&regex);
            MUTEX_UNLOCK();
        }
    }
    vTaskDelete(NULL);
}

scanner_status_t scanner_app_init(scanner_app_cb user_cb)
{
    ESP_LOGI(TAG, "Scanner app init!");

    app_cb = user_cb;

    scanner_mutex = xSemaphoreCreateMutex();
    configASSERT(scanner_mutex);

    scanner_uart_init();
    return hw1258_init();
}

void scanner_app_start(void)
{
    run_flag = 1;
    xTaskCreate(scanner_app_task, "scanner-task", 1024 * 2, NULL, 6, NULL);
}

void scanner_app_stop(void)
{
    run_flag = 0;
}

void scanner_app_deinit(void)
{
    scanner_uart_deinit();
}

scanner_status_t scanner_app_trigger(void)
{
    scanner_uart_write_bytes(SCAN, strlen(SCAN));
    vTaskDelay(pdMS_TO_TICKS(10));

    return hw1258_get_expected_response();
}

scanner_status_t scanner_app_sleep(void) {
    scanner_uart_write_bytes(SLEEP, strlen(SLEEP));
    vTaskDelay(pdMS_TO_TICKS(10));

    return hw1258_get_expected_response();
}

void scanner_app_get_data(char* data)
{
    MUTEX_LOCK();
    strcpy(data, cur_uuid);    
    MUTEX_UNLOCK();
}
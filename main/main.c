#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wpa2.h"
#include "thermal_spi.h"
#include "MelDIR.h"
#include "slave.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/i2c.h"
#include "esp_sntp.h"
#include "esp_attr.h"
#include "esp_event.h"

#include "smart_wifi.h"
#include "ssd1306.h"
#include "vl53l3cx.h"
#include "d6t44l.h"
#include "cloud_api.h"

#define TAG                 "main-app"
#define I2C_MASTER_PORT     0

static uint8_t obtain_time = 1;
char serial[32] = {0};

static void display_time()
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Current Time: %s", strftime_buf);
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronizaion event");
}

static void d6t44lc_event_handler(d6t44l_event_t *evt)
{
    int i;
    switch (evt->id)
    {
    case TEMP_EVT_DATA_READY:
        D6T44_app_stop();
        D6T44_update_temp();
        break;

    case TEMP_EVT_ERROR:

        break;

    default:
        break;
    }
}

static void vl53l3cx_event_handler(vl53l3cx_event_t *evt)
{
    switch (evt->id)
    {
    case TOF_EVT_THRESHOLD_INSIDE:
        D6T44_app_run();
        break;

    case TOF_EVT_THRESHOLD_OUTSIDE:        
        D6T44_reset();
        break;

    default:
        ESP_LOGE(TAG, "Unknow TOF event.");
        break;
    }
}

static void system_info_task(void *pdata)
{
    (void)pdata;

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        last_wake_time = xTaskGetTickCount();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(30000));
        uint32_t iram_free = heap_caps_get_free_size(MALLOC_CAP_32BIT);
        uint32_t uptime = esp_timer_get_time() / 1000000;

        display_time();

        ESP_LOGI(TAG, "RAM32: %u  Uptime: %u seconds", iram_free, uptime);
    }
    vTaskDelete(NULL);
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    sntp_init();
}

static void cloud_cb(cloud_event_t *evt)
{
    static int retries = 0;
    switch (evt->id)
    {
    case CLOUD_EVT_CONNECT:
        ESP_LOGI(TAG, "CLOUD_EVT_CONNECT");
        break;

    case CLOUD_EVT_DISCONNECT:
        ESP_LOGI(TAG, "CLOUD_EVT_DISCONNECT");
        break;

    case CLOUD_EVT_RECONNECT:
        ESP_LOGI(TAG, "CLOUD_EVT_RECONNECT");
        break;

    case CLOUD_EVT_DATA_MQTT:
        ESP_LOGI(TAG, "CLOUD_EVT_DATA_MQTT");
        break;

    default:
        ESP_LOGI(TAG, "UNKNOW_CLOUD_EVENT");
        break;
    }
}

static void connect_to_cloud(void)
{
    cloud_api_init(cloud_cb, CLOUD_TYPE_MQTT);

    cloud_api_set_mqtt_id(serial);

    cloud_ret_t cloud_ret = cloud_api_connect_host("mq.xeleqt.com", 1883);

    if (cloud_ret == CLOUD_RET_OK)
    {
        ESP_LOGI(TAG, "Cloud Connected\r\n");
    }
    else if (cloud_ret == CLOUD_RET_UNKNOWN_PROTOCOL)
    {
        ESP_LOGI(TAG, "Cloud Unknown Protocol\r\n");
    }
}

static void smart_wifi_cb(smart_wifi_event_t *evt)
{
    switch (evt->id)
    {
    case EVT_CONNECT:
        connect_to_cloud();
        if (obtain_time == 1)
        {
            obtain_time = 0;
            initialize_sntp();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            int retry = 0;
            const int retry_count = 10;

            while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count)
            {
                ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            display_time();
        }
        break;

    case EVT_DISCONNECT:

        break;

    default:
        break;
    }
}

static uint8_t i2c_slave_knock(uint8_t i2c_port, uint8_t slave_addr)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(i2c_port, cmd, 100);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

static void i2c_scan(void)
{
    uint8_t slave_count = 0;
    vTaskDelay(500 / portTICK_RATE_MS);
    for (int slave_addr = 0; slave_addr < 127; slave_addr++)
    {
        if (i2c_slave_knock(I2C_MASTER_PORT, slave_addr))
        {
            // Count the number of slaves
            ESP_LOGI(TAG, "Slave_%d_addr %02X", slave_count, slave_addr);
            // Increment count
            slave_count++;
        }
    }

    ESP_LOGI(TAG, "Slave Count %d", slave_count);
}

static void system_init(void)
{
    ESP_LOGI(TAG, "System Init");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    setenv("TZ", "PST-8", 1);
    tzset();

    i2c_config_t i2c_master_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        }};

    i2c_param_config(I2C_MASTER_PORT, &i2c_master_config);
    err = i2c_driver_install(I2C_MASTER_PORT, i2c_master_config.mode, 0, 0, 0);
}

void app_main()
{
    ESP_LOGI(TAG, "Start!");

    system_init();
    i2c_scan();

    uint8_t chipId[6];
    esp_efuse_mac_get_default(chipId);
    sprintf(serial, "%d%d%d%d%d%d", chipId[0], chipId[1], chipId[2], chipId[3], chipId[4], chipId[5]);

    ESP_LOGI(TAG, "%s", serial);

    initialise_wifi(smart_wifi_cb);

    xTaskCreate(system_info_task, "sys-info", 2048, NULL, 1, NULL);

    ssd1306_init();

    // ssd1306_fill(Black);
    // ssd1306_draw_rectangle(1, 1, SSD1306_WIDTH - 1, SSD1306_HEIGHT - 1, White);

    D6T44l_init(d6t44lc_event_handler);

    if (init_vl53l3cx(vl53l3cx_event_handler) == VL53LX_ERROR_NONE)
    {
        vl53l3cx_start_app();
    }
}

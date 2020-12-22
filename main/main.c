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
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/i2c.h"
#include "esp_sntp.h"
#include "esp_attr.h"
#include "esp_event.h"

#include "telemetry.h"
#include "smart_wifi.h"
#include "ssd1306.h"

#include "d6t44l.h"
#include "cloud_api.h"
#include "bluetooth_api.h"
#include "telemetry_protocol.h"
#include "scanner_app.h"

#define VL53L1

#ifdef VL53L1
#include "vl53l1.h"
#else
#include "vl53l3cx.h"
#endif

#define TAG "main-app"
#define I2C_MASTER_PORT 0
// #define ONLINE

#define PORT 1883
#ifdef ONLINE
#define HOST "52.221.96.155"
#else
#define HOST "192.168.10.4"
#endif

#define LED_BLUE 18
#define LED_GREEN 33
#define LED_RED 25
#define BUZZER_IO 19

typedef enum {
    STATE_READY,
    STATE_SCAN,
    STATE_ERROR,
    STATE_AUTH_SUCCESS,
    STATE_AUTH_FAIL,
    STATE_TEMP_RESULT,
} state_t;

static uint8_t sent = 0;

static state_t state = STATE_ERROR;

static uint8_t uuid[16];
static uint8_t obtain_time = 1;
char serial[32] = {0};
uint8_t chipId[6] = {0};

static uint8_t device_type = 0;

static void display_time()
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];

    // strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    sprintf(strftime_buf, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    ESP_LOGI(TAG, "Current Time: %s", strftime_buf);
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronizaion event");
    display_time();
}

static void convert_to_hex_str(char *str, uint8_t *val, size_t len)
{
    sprintf(str, "%02x%02x%02x%02x%02x%02x", val[0], val[1], val[2], val[3], val[4], val[5]);
}

static void reset_led() {
    gpio_set_level(LED_BLUE, 0);
    gpio_set_level(LED_GREEN, 0);
    gpio_set_level(LED_RED, 0);
}

static void gatts_event_handler(bt_gatt_event_t *event)
{
    char instanceID[BT_EDDYSTONE_INSTANCE_LEN * 2] = {0};

    switch (event->id)
    {
    case BT_GATTC_EDDYSTONE:
        if (event->client.event.eddystone.type == BT_EDDYSTONE_TYPE_UID)
        {
            if (event->client.rssi > -65 && sent == 0)
            {
                convert_to_hex_str(instanceID, event->client.event.eddystone.frame.uid.instance, BT_EDDYSTONE_INSTANCE_LEN);
                ESP_LOGI(TAG, "==========================================Eddystone UID Start:=====================================================");
                ESP_LOGI(TAG, "Eddystone Found");
                esp_log_buffer_hex(TAG, event->client.bda, BT_BDA_LEN);
                ESP_LOGI(TAG, "Client RSSI: %d dbm", event->client.rssi);
                ESP_LOGI(TAG, "Measured power(RSSI at 0m distance):%d dbm", event->client.event.eddystone.frame.uid.ranging_data);
                ESP_LOGI(TAG, "Namespace ID:");
                esp_log_buffer_hex(TAG, event->client.event.eddystone.frame.uid.namespace, BT_EDDYSTONE_NAMESPACE_LEN);
                ESP_LOGI(TAG, "Instance ID:");
                ESP_LOGI(TAG, "%s", instanceID);
                ESP_LOGI(TAG, "==========================================Eddystone UID End:=======================================================");

                device_type = 2;
#if 0
                uint8_t buffer[32] = {0};

                telemetry_t *ble_telemetry = (telemetry_t *)buffer;

                ble_telemetry->type = TELEMETRY_TYPE_AUTH;
                ble_telemetry->device_type = TELEMETRY_DEVICE_BLE;
                ble_telemetry->reqNo = reqNo;
                ble_telemetry->serial = strtoull(serial, NULL, 0);

                // buffer[0] = 1;
                memcpy(&ble_telemetry->namespaceID, event->client.event.eddystone.frame.uid.namespace, 10);
                memcpy(&ble_telemetry->instanceID, event->client.event.eddystone.frame.uid.instance, 6);

                uint16_t *crc = _CRC16(buffer, sizeof(ble_telemetry), 0xffff, 0x00);
                memcpy(buffer + sizeof(telemetry_t), &crc, 2);
                cloud_api_send(&buffer, sizeof(telemetry_t) + 2);

                sent = 1;
#endif
            }
        }
        break;

    default:
        break;
    }
}

static void scanner_event_handler(scanner_event_t *evt)
{
    switch (evt->id)
    {
    case SCANNER_EVT_UUID_VALID: // process authentication.
        // ESP_LOGI(TAG, "%s", evt->data);
        device_type = 1;
        D6T44L_start_sampling();
        break;

    case SCANNER_EVT_UUID_INVALID: // play error blink red twice
        state = STATE_ERROR;
        break;

    case SCANNER_EVT_FAIL:
        state = STATE_READY;
        break;

    default:
        break;
    }
}

static void d6t44lc_event_handler(d6t44l_event_t *evt)
{
    int i;
    switch (evt->id)
    {
    case TEMP_EVT_DATA_READY:
        gpio_set_level(LED_BLUE, 0);
        D6T44L_update_temp();
        telemetry_notify_log(device_type);
        // double temp = 0.0;
        // D6T44l_get_data(&temp);
        // ESP_LOGI(TAG, "Temperature: %04.1f", temp);
        break;

    case TEMP_EVT_ERROR:
        state = STATE_ERROR;
        break;

    case TEMP_EVT_TEMP_PASS:
        gpio_set_level(LED_GREEN, 1);
        gpio_set_level(BUZZER_IO, 1);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 0);
        break;
    case TEMP_EVT_RESET:
        // gpio_set_level(LED_BLUE, 1);
        gpio_set_level(LED_GREEN, 0);
        gpio_set_level(LED_RED, 0);
        state = STATE_READY;
        break;

    case TEMP_EVT_TEMP_FAIL:
        gpio_set_level(LED_RED, 1);
        gpio_set_level(BUZZER_IO, 1);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 0);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 1);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 0);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 1);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 0);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 1);
        vTaskDelay(250 / portTICK_RATE_MS);
        gpio_set_level(BUZZER_IO, 0);
        break;

    default:
        break;
    }
}

static void vl53l1_event_handler(vl53l1_event_t *evt)
{
    switch (evt->id)
    {
    case EVT_THRESHOLD_INSIDE:
        state = STATE_SCAN;
        gpio_set_level(LED_BLUE, 1);
        scanner_app_trigger();
        break;

    case EVT_THRESHOLD_OUTSIDE:
        D6T44L_reset();
        scanner_app_sleep();
        state = STATE_READY;
        break;

    default:

        break;
    }
}

#ifndef VL53L1
static void vl53l3cx_event_handler(vl53l3cx_event_t *evt)
{
    switch (evt->id)
    {
    case TOF_EVT_THRESHOLD_INSIDE:
        D6T44L_start_sampling();
        break;

    case TOF_EVT_THRESHOLD_OUTSIDE:
        D6T44L_reset();
        break;

    default:
        ESP_LOGE(TAG, "Unknow TOF event.");
        break;
    }
}
#endif

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
        // Init Bluetooth
        break;

    case CLOUD_EVT_DISCONNECT:
        ESP_LOGI(TAG, "CLOUD_EVT_DISCONNECT");
        break;

    case CLOUD_EVT_RECONNECT:
        ESP_LOGI(TAG, "CLOUD_EVT_RECONNECT");
        break;

    case CLOUD_EVT_DATA_MQTT:
        ESP_LOGI(TAG, "CLOUD_EVT_DATA_MQTT");
        sent = 0;

        ESP_LOGI(TAG, "MQTT Data received");
        ESP_LOGI(TAG, "Topic=%.*s", evt->evt.data.mqtt.topic_len, evt->evt.data.mqtt.topic);
        ESP_LOGI(TAG, "Data=%.*s", evt->evt.data.mqtt.len, evt->evt.data.mqtt.data);
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

    cloud_ret_t cloud_ret = cloud_api_connect_host(HOST, PORT);

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

static void blink_task(void *pvParam)
{
    int cnt = 0;    

    while (1) {
        if (state == STATE_READY) {
            gpio_set_level(LED_BLUE, cnt % 2);
            cnt++;
        }
        vTaskDelay(800 / portTICK_PERIOD_MS);
    }
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

    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);

    gpio_set_direction(BUZZER_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_IO, 0);

    esp_efuse_mac_get_default(chipId);
    sprintf(serial, "%d%d%d%d%d%d", chipId[0], chipId[1], chipId[2], chipId[3], chipId[4], chipId[5]);

    ESP_LOGI(TAG, "%s", serial);

    ESP_LOGI(TAG, "Long serial: %lld", strtoll(serial, NULL, 0));

    initialise_wifi(smart_wifi_cb);

    telemetry_init();

#if 0
    char bluetooth_name[23] = {0};
    sprintf(bluetooth_name, "Xeleqt_%.*s", 6, serial);
    // printf("%s", bluetooth_name);
    bluetooth_register_gatt_handler(gatts_event_handler);
    bluetooth_init(bluetooth_name);
#endif

    if (scanner_app_init(scanner_event_handler) == SCANNER_STATUS_OK)
    {
        scanner_app_start();
    }
    else
    {
        ESP_LOGE(TAG, "Unable to start scanner!");
    }

    ssd1306_init();

    if (D6T44L_init(d6t44lc_event_handler) == 1)
    {
        D6T44L_app_run();
    }
#ifdef VL53L1
    if (vl53l1_init(vl53l1_event_handler) == ESP_OK)
    {
        vl53l1_start_app();
    }
#else
    if (init_vl53l3cx(vl53l3cx_event_handler) == VL53LX_ERROR_NONE)
    {
        vl53l3cx_start_app();
    }
#endif

    xTaskCreate(system_info_task, "sys-info", 2048, NULL, 1, NULL);
    xTaskCreate(blink_task, "blink", 1024, NULL, 5, NULL);
    uint64_t id = strtoull(serial, NULL, 0);
    telemetry_start(&id);
}

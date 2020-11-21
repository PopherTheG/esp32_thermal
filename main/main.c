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

#include "smart_wifi.h"
#include "ssd1306.h"
#include "vl53l3cx.h"
#include "d6t44l.h"
#include "cloud_api.h"
#include "bluetooth_api.h"
#include "ble_telemetry.h"

#define TAG "main-app"
#define I2C_MASTER_PORT 0

#define PORT    1883
#define HOST    "52.221.96.155"

static uint8_t sent = 0;

static uint8_t uuid[16];
static uint8_t obtain_time = 1;
char serial[32] = {0};
uint8_t chipId[6] = {0};
static uint16_t reqNo = 1;

static const uint16_t crc_table[256] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
                                        0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
                                        0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
                                        0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
                                        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
                                        0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
                                        0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
                                        0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
                                        0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
                                        0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
                                        0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
                                        0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
                                        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
                                        0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
                                        0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
                                        0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
                                        0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
                                        0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
                                        0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
                                        0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                                        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
                                        0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
                                        0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
                                        0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
                                        0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
                                        0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
                                        0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
                                        0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
                                        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};



static uint16_t _CRC16(const uint8_t *data, size_t length, uint16_t seed, uint16_t final)
{
    size_t count;
    unsigned int crc = seed;
    unsigned int temp;
    for (count = 0; count < length; ++count)
    {
        temp = (*data++ ^ (crc >> 8)) & 0xff;
        crc = crc_table[temp] ^ (crc << 8);
    }
    return (uint16_t)(crc ^ final);
}

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

static void convert_to_hex_str(char* str, uint8_t* val, size_t len) {
    sprintf(str, "%02x%02x%02x%02x%02x%02x", val[0], val[1], val[2], val[3], val[4], val[5]);
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
                ESP_LOGI(TAG, "==========================================Eddystone UID End:=====================================================");

                uint8_t buffer[32] = {0};

                ble_telemetry_t *ble_telemetry = (ble_telemetry_t *)buffer;

                ble_telemetry->type = 1;
                ble_telemetry->reqNo = reqNo;
                ble_telemetry->serial = strtoull(serial, NULL, 0);

                // buffer[0] = 1;
                memcpy(&ble_telemetry->namespaceID, event->client.event.eddystone.frame.uid.namespace, 10);
                memcpy(&ble_telemetry->instanceID, event->client.event.eddystone.frame.uid.instance, 6);                
                cloud_api_send(&buffer, sizeof(ble_telemetry_t));

                sent = 1;
            }
        }
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
        D6T44L_start_sampling();
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
    
    esp_efuse_mac_get_default(chipId);
    sprintf(serial, "%d%d%d%d%d%d", chipId[0], chipId[1], chipId[2], chipId[3], chipId[4], chipId[5]);

    ESP_LOGI(TAG, "%s", serial);

    ESP_LOGI(TAG, "Long serial: %lld", strtoll(serial, NULL, 0));

    initialise_wifi(smart_wifi_cb);

    char bluetooth_name[23] = {0};
    sprintf(bluetooth_name, "Xeleqt_%.*s", 6, serial);
    printf("%s", bluetooth_name);
    bluetooth_register_gatt_handler(gatts_event_handler);
    bluetooth_init(bluetooth_name);

    xTaskCreate(system_info_task, "sys-info", 2048, NULL, 1, NULL);

    ssd1306_init();

    if (D6T44l_init(d6t44lc_event_handler) == 1)
    {
        D6T44_app_run();
    }

    if (init_vl53l3cx(vl53l3cx_event_handler) == VL53LX_ERROR_NONE)
    {
        vl53l3cx_start_app();
    }
}

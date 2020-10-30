#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_smartconfig.h"
#include "cloud_api.h"
#include "thermal_spi.h"
#include "MelDIR.h"
#include "slave.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/i2c.h"
#include "ssd1306_tests.h"

#include "d6t44l.h"

#define TAG "main-app"

#define I2C_MASTER_PORT 0

static EventGroupHandle_t s_wifi_event_group;

static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;

static thermal_img_t tImg;
static bool load_nvs_success = false;

static void smartconfig_example_task(void *parm);

static void save_config_to_nvs(wifi_config_t *config)
{
    esp_err_t err;
    nvs_handle_t handle;

    err = nvs_open("storage", NVS_READWRITE, &handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    err = nvs_set_str(handle, "ssid", (char *)config->sta.ssid);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    err = nvs_set_str(handle, "password", (char *)config->sta.password);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    nvs_close(handle);
}

static esp_err_t load_config_from_nvs(wifi_config_t *config)
{
    esp_err_t err;
    nvs_handle_t handle;

    size_t len = 32;
    err = nvs_open("storage", NVS_READWRITE, &handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    err = nvs_get_str(handle, "ssid", (char *)config->sta.ssid, &len);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    size_t pLen = 32;
    err = nvs_get_str(handle, "password", (char *)config->sta.password, &pLen);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    nvs_close(handle);

    return err;
}

static void cloud_cb(cloud_event_t *evt)
{
    static int retries = 0;
    switch (evt->id)
    {
    case CLOUD_EVT_CONNECT:
        ESP_LOGI(TAG, "Connected to Cloud!");
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
        ESP_LOGI(TAG, "UNKNOW CLOUD EVENT");
        break;
    }
}

static void ir_event_handler(ir_event_t *event)
{
    switch (event->id)
    {

    case IR_EVENT_WAKEUP_SUCCESS:
        ESP_LOGI(TAG, "IR_EVENT_WAKEUP_SUCCESS");
        break;

    case IR_EVENT_WAKEUP_FAIL:
        ESP_LOGI(TAG, "IR_EVENT_WAKEUP_FAIL");
        break;

    case IR_EVENT_INIT_SUCCES:
        ESP_LOGI(TAG, "IR_EVENT_INIT_SUCCES");
        break;

    case IR_EVENT_INIT_FAIL:
        ESP_LOGI(TAG, "IR_EVENT_INIT_FAIL");
        break;

    case IR_EVENT_SHUTTER_SUCCES:
        ESP_LOGI(TAG, "SHUTTER SUCCESS");
        vTaskDelay(500 / portTICK_RATE_MS);
        thermal_start();
        break;

    case IR_EVENT_SHUTTER_FAIL:
        ESP_LOGI(TAG, "SHUTTER FAILED");
        break;

    case IR_EVENT_THERMAL_IMG_FAIL:
        thermal_stop();
        vTaskDelay(500 / portTICK_RATE_MS);
        thermal_start();
        break;

    case IR_EVENT_THERMAL_IMG_SUCCES:
    {

        break;
    }

    default:
        ESP_LOGE(TAG, "UNKNOWN");
        break;
    }
}

static void connect_to_cloud(void)
{
    cloud_api_init(cloud_cb, CLOUD_TYPE_MQTT);
    cloud_api_set_mqtt_id("esp_thermal_client");

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

static void smartconfig_example_task(void *parm)
{
    ESP_LOGI(TAG, "smartconfig task start");
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1)
    {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        if (load_nvs_success)
        {
            esp_wifi_connect();
        }
        else
        {
            xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 2048 * 2, NULL, 3, NULL);
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        connect_to_cloud();
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE)
    {
        ESP_LOGI(TAG, "Scan done");
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL)
    {
        ESP_LOGI(TAG, "Found channel");
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD)
    {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = {0};
        uint8_t password[65] = {0};

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true)
        {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);

        save_config_to_nvs(&wifi_config);

        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_connect());

        load_config_from_nvs(&wifi_config);
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE)
    {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
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

    i2c_config_t i2c_master_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,                
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        }
    };

    i2c_param_config(I2C_MASTER_PORT, &i2c_master_config);
    err = i2c_driver_install(I2C_MASTER_PORT, i2c_master_config.mode, 0, 0, 0);    

    uint8_t slave_count = 0;

    vTaskDelay(500 / portTICK_RATE_MS);

    for (int slave_addr = 0; slave_addr < 127; slave_addr++) {
        if (i2c_slave_knock(I2C_MASTER_PORT, slave_addr)) {
            // Count the number of slaves
            ESP_LOGI(TAG, "Slave_%d_addr %02X",slave_count, slave_addr);
            // Increment count
            slave_count++;
        }
    }

    ESP_LOGI(TAG, "Slave Count %d", slave_count);
}

static void initialise_wifi(void)
{
    ESP_LOGI(TAG, "WIFI Init");
    wifi_config_t sta_config = {0};

    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    if (ESP_OK == load_config_from_nvs(&sta_config))
    {
        ESP_LOGI(TAG, "Init SSID:%s", sta_config.sta.ssid);
        ESP_LOGI(TAG, "Init PASSWORD:%s", sta_config.sta.password);
        load_nvs_success = true;

        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_config));
    }   

    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main()
{
    ESP_LOGI(TAG, "Start!");

    system_init();

    // if(D6T44l_init() == 1) {
    //     app_run();
    // }

    // vTaskDelay(1000 / portTICK_RATE_MS);
    // initialise_wifi();

    // ssd1306_init();
    // ssd1306_TestAll();
    // D6T44l_init();
}

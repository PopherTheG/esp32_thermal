#include <stdio.h>
#include <time.h>
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

#include "smart_wifi.h"
#include "ssd1306.h"
#include "vl53l3cx.h"
#include "d6t44l.h"

#define TAG "main-app"

#define I2C_MASTER_PORT 0

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

    // if(D6T44l_init() == 1) {
    //     app_run();
    // }
    // vTaskDelay(1000 / portTICK_RATE_MS);
    initialise_wifi();

    // ssd1306_init();
    // D6T44l_init();
    if (init_vl53l3cx() == VL53LX_ERROR_NONE)
    {
        vl53Ll3cx_start_app();
    }
}

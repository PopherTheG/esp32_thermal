#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "d6t44l.h"
#include "ssd1306.h"
#include <math.h>

#define WRITE_ADDR ((D6T_ADDR << 1) | I2C_MASTER_WRITE)
#define READ_ADDR ((D6T_ADDR << 1) | I2C_MASTER_READ)

uint8_t rbuf[N_READ];
double pix_data[N_PIXEL];

static void updateScreen(int16_t temp) {
    char str[20];

    sprintf(str, "%d C", temp);
    ssd1306_fill(Black);
    ssd1306_set_cursor(20, 20);

    ssd1306_write_string(str, Font_16x26, White);
    ssd1306_update_screen();
}

static int scan_device(void)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, WRITE_ADDR, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK)
        return 1;

    return 0;
}

uint8_t calc_crc(uint8_t data)
{
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++)
    {
        temp = data;
        data <<= 1;
        if (temp & 0x80)
        {
            data ^= 0x07;
        }
    }
    return data;
}

bool D6T_checkPEC(uint8_t buf[], int n)
{
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1); // I2C Read address (8bit)
    for (i = 0; i < n; i++)
    {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret)
    {
        ESP_LOGE(__FUNCTION__, "PEC check Failed: %02X(cal)-%02x(get)\n", crc, buf[n]);
    }
    return ret;
}

int16_t conv8us_s16_le(uint8_t *buf, int n)
{
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret; // and convert negative.
}

static int i2c_write(void)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, WRITE_ADDR, 1);
    i2c_master_write_byte(cmd, D6T_CMD, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

static int i2c_read(uint8_t *val, size_t len)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, READ_ADDR, 1);
    i2c_master_read(cmd, val, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

static void d6t_app_task(void)
{
    while (1)
    {
        memset(rbuf, 0, N_READ);
        int i, j;

        int ret;
        ret = i2c_write();
        vTaskDelay(10 / portTICK_RATE_MS);
        ret = i2c_read(rbuf, N_READ);
        if (ret == 0)
        {
            if (!D6T_checkPEC(rbuf, N_READ - 1))
            {
                int16_t itemp = conv8us_s16_le(rbuf, 0);           
                // ESP_LOGI(__FUNCTION__, "PTAT: %4.1f [degC], Temperature: ", itemp / 10.0);
                int16_t hTemp = 0;
                for ( i = 0, j = 2; i < N_PIXEL; i++, j+=2 ) {
                   int16_t temp = conv8us_s16_le(rbuf, j) / 10.0;
                   pix_data[i] = temp;
                   if (temp > hTemp) {
                       hTemp = temp;
                   }
                }

                // ESP_LOGI(__FUNCTION__, "Highest Temp: %f", floor(hTemp));
                updateScreen(hTemp);
            }
        }

        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void app_run(void)
{
    xTaskCreate(d6t_app_task, "d6t-task", 2048, NULL, 5, NULL);
}

int D6T44l_init(void)
{
    int err = scan_device();

    if (err != 1) {
        ESP_LOGE(__FUNCTION__, "Device Not found");
        return -1;
    }

    app_run();
    return err;
}
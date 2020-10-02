#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "thermal_spi.h"
#include "MelDIR.h"

#define TAG     "thermal"

#define MUTEX_TIMEOUT_MS    50
#define IR_MUTEX_LOCK()     xSemaphoreTake(ir_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS));
#define IR_MUTEX_UNLOCK()   xSemaphoreGive(ir_mutex)

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

spi_device_handle_t spi;

static sensor_into_reg_t sensor_info_reg;
static frame_reg_t frame_reg;
static thermal_img_t thermal_img;
static uint8_t run_flag = 0;
static SemaphoreHandle_t ir_mutex;
static ir_app_cb app_event_handler;
static uint8_t ir_present = 0;

static float temp_brd_bin_2_float(uint16_t temp_brd, uint8_t ther_offset) {
    float x = (float) 4.5 * (1 - ( (float)temp_brd - (float) ther_offset )/1024.0);
    return (float) -19.47 * x + 75.04;
}

static void thermal_image_bin_2_float( thermal_img_t *img, float tref,  temp_img_t *tImg) {
    uint8_t x;
    uint8_t y;
    float max_temp = 0.0;

    tImg->Tbrd = tref;

    for ( y = 0; y < 80; y++ ) {
        for ( x = 0; x < 32; x++ ) {
            tImg->Tpixel[x][y] = (float)(img->pixel[x][y] - 8191) / 30.0 + 25.5;
            if ( tImg->Tpixel[x][y] > max_temp ) {
                max_temp = tImg->Tpixel[x][y];
            }
        }
    }

    tImg->max_temp = max_temp;
}

static void thermal_app_task(void* pData) {    
    while(run_flag) {
        int ret = GetThermalImage(&frame_reg, &thermal_img);
        if (ret == ERROR) {            
            ir_event_t event = {0};
            event.id = IR_EVENT_THERMAL_IMG_FAIL;            
            app_event_handler(&event);
        } else {
            ir_event_t event = {0};
            event.id = IR_EVENT_THERMAL_IMG_SUCCES;            
            app_event_handler(&event);
        }
        vTaskDelay(500 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}


int init_thermal(ir_app_cb event_handler) {

    esp_err_t esp_err = ESP_FAIL;
    int ret = 0;

    ir_mutex = xSemaphoreCreateMutex();
    configASSERT(ir_mutex != NULL);

    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);

    spi_bus_config_t buscfg={
            .miso_io_num = PIN_NUM_MISO,
            .mosi_io_num = PIN_NUM_MOSI,
            .sclk_io_num = PIN_NUM_CLK, 
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 512
    };
    
    spi_device_interface_config_t devcfg={
            .clock_speed_hz = 1000000,
            .mode = 2,
            .spics_io_num = -1,
            .queue_size = 7,
    };

    app_event_handler = event_handler;

    esp_err = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (esp_err != OK)
        ret = ERROR;

    esp_err = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if(esp_err == OK) {
        ir_present = 1;
    } else {
        ret =  ERROR;
    }        

    gpio_set_level(PIN_NUM_CS, 0);
    vTaskDelay(150/portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_CS, 1);
    vTaskDelay(150/portTICK_RATE_MS);
 
    ret = WakeUP(spi);
    if (ret != SUCCESS) {
        ir_event_t event = {0};
        event.id = IR_EVENT_WAKEUP_FAIL;
        app_event_handler(&event);
        ret = ERROR;
    } else {
        ir_event_t event = {0};
        event.id = IR_EVENT_WAKEUP_SUCCESS;
        app_event_handler(&event);
    }

    ret = INIT_sensor_sc(&sensor_info_reg, 0);
    if(ret != SUCCESS) {
        ir_event_t event = {0};
        event.id = IR_EVENT_INIT_FAIL;
        app_event_handler(&event);
        ret = ERROR;
    } else {
        ir_event_t event = {0};
        event.id = IR_EVENT_INIT_SUCCES;
        app_event_handler(&event);
    }

    ret = ShutterAct(&frame_reg);
    if(ret != SUCCESS) {
        ir_event_t event = {0};
        event.id = IR_EVENT_SHUTTER_FAIL;
        app_event_handler(&event);
        ret = ERROR;
    } else {
        ir_event_t event = {0};
        event.id = IR_EVENT_SHUTTER_SUCCES;
        app_event_handler(&event);
    }

    return ret;
}

void thermal_start(void) {
    run_flag = 1;
    xTaskCreate(thermal_app_task, "thermal-task", 1024, NULL, 5, NULL);
}

void thermal_stop(void) {
    run_flag = 0;
}

void thermal_app_deinit(void ) {
    spi_bus_remove_device(spi);
    spi_bus_free(HSPI_HOST);
}

void thermal_get_data(thermal_img_t *data) {
    
    if(ir_present) {
        IR_MUTEX_LOCK();
        data->brd = thermal_img.brd;        
        memcpy(data->pixel, thermal_img.pixel, sizeof(thermal_img.pixel));        
        IR_MUTEX_UNLOCK();
    }
}
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <string.h>
#include <stdlib.h>
#include "vl53lx_api.h"

#define TAG "vl53l3cx-app"

VL53LX_Dev_t dev;
VL53LX_DEV Dev = &dev;

static uint8_t run_flag = 0;

static void vl53l3cx_app_task(void *arg)
{
    // VL53LX_DEV Dev = (VL53LX_DEV)arg;

    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;

    VL53LX_Error status = VL53LX_ERROR_NONE;

    do
    {
        status = VL53LX_GetMeasurementDataReady(Dev, &NewDataReady);
        vTaskDelay(10 / portTICK_RATE_MS);
        if ((!status) && (NewDataReady != 0))
        {
            status = VL53LX_GetMultiRangingData(Dev, pMultiRangingData);
            no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
            if (no_of_object_found > 0)
            {
                printf("Count=%5d, ", pMultiRangingData->StreamCount);
                printf("#Objs=%1d ", no_of_object_found);
                for (j = 0; j < no_of_object_found; j++)
                {
                    if (j != 0)
                        printf("\n");

                    if (pMultiRangingData->RangeData[j].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID)
                    {
                        printf("status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
                               pMultiRangingData->RangeData[j].RangeStatus,
                               pMultiRangingData->RangeData[j].RangeMilliMeter,
                               pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0,
                               pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
                    }
                }
                printf("\n");
            }

            if (status == 0)
            {
                status = VL53LX_ClearInterruptAndStartMeasurement(Dev);
            }
        }
        vTaskDelay(250 / portTICK_RATE_MS);
    } while (run_flag);
    VL53LX_StopMeasurement(Dev);
    vTaskDelete(NULL);
}

void vl53Ll3cx_start_app(void)
{
    ESP_LOGI(TAG, "Starting Task");
    run_flag = 1;
    xTaskCreate(vl53l3cx_app_task, "app-task", 2048, NULL, 5, NULL);    
}

void vl53l3cx_stop_app(void)
{
    run_flag = 0;
}

VL53LX_Error init_vl53l3cx(void)
{
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_32, 0);

    Dev->i2c_slave_address = 0x29;
    gpio_set_level(GPIO_NUM_32, 1);
    // status = init_device(Dev, 0x43);

    VL53LX_Error status = VL53LX_ERROR_NONE;

    ESP_LOGI(TAG, "Old address: %x", Dev->i2c_slave_address);

    status = VL53LX_WaitDeviceBooted(Dev);
    ESP_LOGI(TAG, "VL53LX_WaitDeviceBooted %d", status);

    status = VL53LX_DataInit(Dev);
    ESP_LOGI(TAG, "VL53LX_DataInit %d", status);

    // status = VL53LX_SetDeviceAddress(Dev, 0x52);
    // if (status == VL53LX_ERROR_NONE)    
    //     Dev->i2c_slave_address = 0x52;
    
    // ESP_LOGI(TAG, "VL53LX_SetDeviceAddress %d new address %x", status, Dev->i2c_slave_address);

    vTaskDelay(10 / portTICK_RATE_MS);    

    status = VL53LX_StartMeasurement(Dev);
    ESP_LOGI(TAG, "VL53LX_StartMeasurement %d", status);

    return status;
}
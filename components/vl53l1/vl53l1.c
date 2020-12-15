#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"

#include "vl53l1.h"

#define TAG "vl53l1"

static uint8_t dev;
static uint8_t run_flag = 0;
static vl53l1_app_cb user_callbacks = NULL;

static int center[2] = {167, 231}; /* these are the spad center of the 2 8*16 zones */
static int Zone = 0;
static int PplCounter = 0;
static uint8_t isInside = 0;

static int process_people_counting_data(int16_t distance, uint8_t zone)
{
    static int PathTrack[] = {0, 0, 0, 0};
    static int PathTrackFillingSize = 1;
    static int LeftPreviousStatus = NOBODY;
    static int RightPreviousStatus = NOBODY;
    static int PeopleCount = 0;

    int CurrentZoneStatus = NOBODY;
    int AllZoneCurrentStatus = 0;
    int AneventHasOccured = 0;

    if (distance < DIST_THRESHOLD_MAX)
    {
        CurrentZoneStatus = SOMEONE;
    }

    if (zone == LEFT)
    {
        if (CurrentZoneStatus != LeftPreviousStatus)
        {
            AneventHasOccured = 1;
            if (CurrentZoneStatus == SOMEONE)
            {
                AllZoneCurrentStatus += 1;
            }

            if (RightPreviousStatus == SOMEONE)
            {
                AllZoneCurrentStatus += 2;
            }
            LeftPreviousStatus = CurrentZoneStatus;
        }
    }
    else
    {
        if (CurrentZoneStatus != RightPreviousStatus)
        {
            AneventHasOccured = 1;
            if (CurrentZoneStatus == SOMEONE)
            {
                AllZoneCurrentStatus += 2;
            }
            if (LeftPreviousStatus == SOMEONE)
            {
                AllZoneCurrentStatus += 1;
            }

            RightPreviousStatus = CurrentZoneStatus;
        }
    }

    if (AneventHasOccured)
    {
        if (PathTrackFillingSize < 4)
        {
            PathTrackFillingSize++;
        }

        if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY))
        {
            if (PathTrackFillingSize == 4)
            {
                ESP_LOGI(TAG, "%d, %d, %d, %d", PathTrack[0], PathTrack[1], PathTrack[2], PathTrack[3]);

                if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2))
                {
                    PeopleCount++;
                }
                else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1))
                {
                    PeopleCount--;
                }
            }
            PathTrackFillingSize = 1;
        }
        else
        {
            PathTrack[PathTrackFillingSize - 1] = AllZoneCurrentStatus;
        }
    }

    return (PeopleCount);
}

static void vl53l1x_app_task(void *arg)
{
    uint16_t Distance;
    uint8_t RangeStatus;
    uint8_t dataReady;

    uint8_t status = 0;

    while (run_flag)
    {
        while (dataReady == 0)
        {
            status = VL53L1X_CheckForDataReady(dev, &dataReady);
            vTaskDelay(2 / portTICK_RATE_MS);
        }

        dataReady = 0;
        status += VL53L1X_GetRangeStatus(dev, &RangeStatus);
        status += VL53L1X_GetDistance(dev, &Distance);
        status += VL53L1X_ClearInterrupt(dev);
        if (status != 0)
        {
            ESP_LOGE(TAG, "Error in operating the device.");
        }

#ifdef PEOPLE_COUNTING_MODE
        status = VL53L1X_SetROICenter(dev, center[Zone]);
        if (status != 0)
        {
            ESP_LOGE(TAG, "Error in changing the center of the ROI.\n");
        }

        PplCounter = process_people_counting_data(Distance, Zone);

        Zone++;
        Zone = Zone % 2;

        ESP_LOGI(TAG, "Zone: %d, RangeStatus: %d, Distance: %d, Count: %d", Zone, RangeStatus, Distance, PplCounter);
#else

        // ESP_LOGI(TAG, "Distance: %d", Distance);
        if (Distance <= DIST_THRESHOLD_MAX && isInside == 0 ) {
            isInside = 1;
            vl53l1_event_t event = {0};
            event.id = EVT_THRESHOLD_INSIDE;
            event.distance = Distance;
            user_callbacks(&event);

        } else if (Distance > DIST_THRESHOLD_MAX && isInside == 1) {
            isInside = 0;
            vl53l1_event_t event = {0};
            event.id = EVT_THRESHOLD_OUTSIDE;
            event.distance = Distance;
            user_callbacks(&event);
        }
#endif

        vTaskDelay(200 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
    VL53L1X_StopRanging(dev);
}

void vl53l1_stop_app(void)
{
    run_flag = 0;
}

void vl53l1_start_app(void)
{
    run_flag = 1;
    xTaskCreate(vl53l1x_app_task, "vl53l1-task", 2048, NULL, 5, NULL);
}

uint8_t vl53l1_init(vl53l1_app_cb app_cb)
{
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_32, 0);

    user_callbacks = app_cb;

    dev = 0x29;

    gpio_set_level(GPIO_NUM_32, 1);

    uint8_t byteData, sensorState = 0;
    uint8_t status;
    uint16_t wordData;

    // VL53L1X_SetI2CAddress();

    status = VL53L1_RdByte(dev, 0x010F, &byteData);
    ESP_LOGI(TAG, "Vl53L1 Model ID: %X", byteData);
    status = VL53L1_RdByte(dev, 0x0110, &byteData);
    ESP_LOGI(TAG, "Vl53L1 Module Type: %X", byteData);
    status = VL53L1_RdWord(dev, 0x010F, &wordData);
    ESP_LOGI(TAG, "Vl53L1: %X", wordData);

    while (sensorState == 0)
    {
        status = VL53L1X_BootState(dev, &sensorState);
        vTaskDelay(200 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Chip booted");

    status = VL53L1X_SensorInit(dev);
    // ESP_LOGE(TAG, "Sensor Init %d", status);
    status += VL53L1X_SetDistanceMode(dev, 2);
    // ESP_LOGE(TAG, "Set Distance Mode %d", status);
    status += VL53L1X_SetTimingBudgetInMs(dev, 20);
    // ESP_LOGE(TAG, "Set Timing Budget %d", status);
    status = VL53L1X_SetInterMeasurementInMs(dev, 100);
    // ESP_LOGE(TAG, "Set Inter Measurement %d", status);
    status += VL53L1X_SetROI(dev, 8, 16);
    // ESP_LOGE(TAG, "Set ROI %d", status);

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in Initialization or configuration of the device");
    }

    VL53L1X_StartRanging(dev);

    return status;
}
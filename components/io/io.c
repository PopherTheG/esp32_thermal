
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_log.h"

#include "TCA6416A.h"
#include "io.h"
#include "TCA6416A_i2c.h"

#define IO_MUTEX_LOCK(_mutex) xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))
#define IO_MUTEX_UNLOCK(_mutex) xSemaphoreGive(_mutex)

#define TAG "io-task"

#define HIGH 1
#define LOW 0

typedef enum {
    TCA6416A_RELAY_SET,    
} output_event_id_t;

typedef struct {
    output_event_id_t id;
    uint8_t level;
} output_event_t;

static SemaphoreHandle_t io_mutex;
static SemaphoreHandle_t io_semaphore;
static QueueHandle_t tca6416a_out_queue;
static uint8_t relay_level = 0;
static TCA6416ARegs regs;
static TCA6416ARegs input_regs;


static void tca6416a_task(void* data) {
    (void)data;
    while (1)
    {
        output_event_t output_evt;
        if(xQueueReceive(tca6416a_out_queue, &output_evt, portMAX_DELAY) == pdPASS) {
            switch (output_evt.id)
            {
            case TCA6416A_RELAY_SET:
                ESP_LOGI(__FUNCTION__, "set relay %d", output_evt.level);
                
                IO_MUTEX_LOCK(io_mutex);
                if(output_evt.level) {
                    relay_level = 0;
                    TCA6416AWriteOutputPin(TCA6416A_P05, HIGH, TCA6416A_ADDRESS);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    TCA6416AWriteOutputPin(TCA6416A_P05, LOW, TCA6416A_ADDRESS);

                } else {
                    relay_level = 1;
                    TCA6416AWriteOutputPin(TCA6416A_P06, HIGH, TCA6416A_ADDRESS);
                    vTaskDelay(pdMS_TO_TICKS(10));
                    TCA6416AWriteOutputPin(TCA6416A_P06, LOW, TCA6416A_ADDRESS);
                }
                IO_MUTEX_UNLOCK(io_mutex);
                break;
            
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void io_init(void)
{
    TCA6416AInitDefault(&regs);

    regs.Config.Port.P0.bit.B0 = TCA6416A_CONFIG_INPUT;

    //Relay
    regs.Config.Port.P0.bit.B5 = TCA6416A_CONFIG_OUTPUT;
    regs.Config.Port.P0.bit.B6 = TCA6416A_CONFIG_OUTPUT;
    


#if 0
    regs.Config.Port.P0.bit.B0 = TCA6416A_CONFIG_OUTPUT;
    regs.Config.Port.P0.bit.B1 = TCA6416A_CONFIG_OUTPUT;
    regs.Config.Port.P0.bit.B2 = TCA6416A_CONFIG_OUTPUT;
    regs.Config.Port.P0.bit.B4 = TCA6416A_CONFIG_OUTPUT; // en_switch
    regs.Config.Port.P0.bit.B5 = TCA6416A_CONFIG_OUTPUT; // buzzer

    regs.Config.Port.P1.bit.B3 = TCA6416A_CONFIG_OUTPUT; // vl53l3 xshut
    regs.Config.Port.P1.bit.B5 = TCA6416A_CONFIG_OUTPUT; // green led
    regs.Config.Port.P1.bit.B6 = TCA6416A_CONFIG_OUTPUT; // blue led
    regs.Config.Port.P1.bit.B7 = TCA6416A_CONFIG_OUTPUT; // red led

    regs.Output.Port.P1.bit.B3 = 1;
    regs.Output.Port.P0.bit.B4 = 0;

#endif

    if (TCA6416AInitI2CReg(&regs, TCA6416A_ADDRESS) == I2C_OPERATION_FAIL) {
        ESP_LOGE(TAG, "Failed to init TCA6416A");
    }

    tca6416a_out_queue = xQueueCreate(10, sizeof(output_event_t));

    io_mutex = xSemaphoreCreateMutex();
    configASSERT(io_mutex);

    xTaskCreate(tca6416a_task, "tca6416a-out", 1024 * 3, NULL, 6, NULL);
}


void io_set_relay(uint8_t level)
{
    output_event_t relay_event = {0};
    relay_event.id = TCA6416A_RELAY_SET;
    relay_event.level = level ? HIGH : LOW;
    xQueueSendToBack(tca6416a_out_queue, &relay_event, pdMS_TO_TICKS(100));
}

uint8_t io_get_relay(void)
{
    return relay_level;
}

void io_vl53l3_pin_enable(uint8_t en)
{
}
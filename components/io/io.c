
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "TCA6416A.h"
#include "io.h"
#include "TCA6416A_i2c.h"

#define IO_MUTEX_LOCK(_mutex) xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))
#define IO_MUTEX_UNLOCK(_mutex) xSemaphoreGive(_mutex)

#define TAG "io-task"

#define HIGH 1
#define LOW 0

static SemaphoreHandle_t io_mutex;

static TCA6416ARegs regs;

void io_init(void)
{
    TCA6416AInitDefault(&regs);

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

    if (TCA6416AInitI2CReg(&regs, TCA6416A_ADDRESS_EXPND) == I2C_OPERATION_FAIL) {
        ESP_LOGE(TAG, "Failed to init TCA6416A");
    }

    io_mutex = xSemaphoreCreateMutex();
    configASSERT(io_mutex);
}

void io_set_red_led(uint8_t state)
{
}

void io_set_blue_led(uint8_t state)
{
}

void io_set_green_led(uint8_t state)
{
}

void io_set_buzzer(uint8_t state)
{
}

void io_set_relay(uint8_t level)
{
}

uint8_t io_get_relay(void)
{
    return 0;
}

void io_vl53l3_pin_enable(uint8_t en)
{
}
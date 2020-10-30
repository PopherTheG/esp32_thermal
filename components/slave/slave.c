#include "esp_log.h"
#include "driver/i2c.h"
#include "cloud_api.h"

#define TAg     "i2c-slave";

#define DATA_LENGTH 512  

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512  

#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_SLAVE_SCL_IO        5               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO        4               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(0)             /*!< I2C port number for slave dev */

#define ESP_SLAVE_ADDR          0x28

SemaphoreHandle_t print_mux = NULL;
uint8_t run_flag = 0;

static void display_buff(uint8_t *buf, int len) {
    cloud_api_send(buf, len);
    
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n================================================\n");
}

static void task_slave(void* pData) {
    int i = 0;
    int ret;
    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
    int size;
    while(run_flag) {
        xSemaphoreTake(print_mux, portMAX_DELAY);
        size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data_rd, DATA_LENGTH, 1000 / portTICK_RATE_MS);        

        if(size > 0) {
            display_buff(data_rd, size);
        }
        xSemaphoreGive(print_mux);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

esp_err_t i2c_slave_init(void) {

    print_mux = xSemaphoreCreateMutex();

    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR,
    };

    i2c_param_config(i2c_slave_port, &conf_slave);

    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN,0,0);    
}

void i2c_start(void) {
    run_flag = 1;
    xTaskCreate(task_slave, "i2c_slave_task", 1024 * 2, NULL, 5, NULL);
}

esp_err_t i2c_slave_deinit(void) {
    return i2c_driver_delete(I2C_SLAVE_NUM);     
}

void ic2_stop(void) {
    run_flag = 0;
}
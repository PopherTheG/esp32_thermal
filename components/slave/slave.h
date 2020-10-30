#ifndef COMPONENTS_SLAVE_SLAVE
#define COMPONENTS_SLAVE_SLAVE

#include "esp_system.h"

esp_err_t i2c_slave_init(void);

esp_err_t i2c_slave_deinit(void);

void i2c_start(void);

void i2c_stop(void);

#endif /* COMPONENTS_SLAVE_SLAVE */

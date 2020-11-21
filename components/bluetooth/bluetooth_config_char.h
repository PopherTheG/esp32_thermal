#ifndef COMPONENTS_BLUETOOTH_BLUETOOTH_CONFIG_CHAR
#define COMPONENTS_BLUETOOTH_BLUETOOTH_CONFIG_CHAR

#include <stdint.h>
#include <stdlib.h>

void bluetooth_config_char_basic_read(uint8_t* out_buff, size_t* out_len);

void bluetooth_config_char_basic_write(const uint8_t* data, size_t len);

void bluetooth_config_char_system_read(uint8_t* out_buff, size_t* out_len);

void bluetooth_config_char_system_write(const uint8_t* data, size_t len);
#endif /* COMPONENTS_BLUETOOTH_BLUETOOTH_CONFIG_CHAR */

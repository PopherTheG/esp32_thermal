#include "driver/i2c.h"
#include "TCA6416A_i2c.h"

#define I2C_ACK_EN 1

int8_t I2C_Write(uint8_t byte_count, uint8_t slave_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t offset)
{
    esp_err_t i2c_err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, I2C_ACK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_EN);
    i2c_master_write(cmd, reg_data, byte_count, I2C_ACK_EN);
    i2c_master_stop(cmd);
    i2c_err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100);
    i2c_cmd_link_delete(cmd);

    return i2c_err == ESP_OK ? 0 : -1;
}

int8_t I2C_Read(uint8_t byte_count, uint8_t slave_addr, uint8_t reg_addr, uint8_t *reg_data)
{
    esp_err_t i2c_err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE,
    I2C_ACK_EN);
    i2c_master_write_byte( cmd, reg_addr, I2C_ACK_EN );
    i2c_master_start( cmd );
    i2c_master_write_byte(cmd,
            ( slave_addr << 1) | I2C_MASTER_READ,
            I2C_ACK_EN);
    i2c_master_read( cmd, reg_data, byte_count, I2C_MASTER_LAST_NACK );
    i2c_master_stop( cmd );
    i2c_err = i2c_master_cmd_begin( I2C_NUM_0, cmd, 100 );
    i2c_cmd_link_delete( cmd );

    return i2c_err == ESP_OK ? 0:-1;
}

uint8_t I2C_Read_Byte(uint8_t slave_addr, uint8_t reg_addr)
{
    uint8_t reg_data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE,
    I2C_ACK_EN);
    i2c_master_write_byte( cmd, reg_addr, I2C_ACK_EN );
    i2c_master_start( cmd );
    i2c_master_write_byte(cmd,
            ( slave_addr << 1) | I2C_MASTER_READ,
            I2C_ACK_EN);
    i2c_master_read_byte( cmd, &reg_data,  I2C_MASTER_LAST_NACK );
    i2c_master_stop( cmd );
    i2c_master_cmd_begin( I2C_NUM_0, cmd, 100 );
    i2c_cmd_link_delete( cmd );

    return reg_data;
}
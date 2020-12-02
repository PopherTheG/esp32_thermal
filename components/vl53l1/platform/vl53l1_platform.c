
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#include "driver/i2c.h"
#include "esp_err.h"

uint8_t _I2CBuffer[256];

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	
	_I2CBuffer[0] = index >> 8;
	_I2CBuffer[1] = index & 0xff;

	memcpy(&_I2CBuffer[2], pdata, count);

	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write(cmd, _I2CBuffer, count + 2, true);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return err; 
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	esp_err_t err;

	_I2CBuffer[0] = index >> 8;
	_I2CBuffer[1] = index & 0xff;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write(cmd, _I2CBuffer, 2, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, 1);
	i2c_master_read(cmd, pdata, count, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	
	return err; 
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	_I2CBuffer[0] = index >> 8;
	_I2CBuffer[1] = index & 0xff;
	_I2CBuffer[2] = (uint8_t)(data);

	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write(cmd, _I2CBuffer, 3, true);	
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 200 / portTICK_PERIOD_MS);	
	i2c_cmd_link_delete(cmd);

	return err; 
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	uint8_t buffer[2];

	buffer[0] = (uint8_t)(data >> 8);
	buffer[1] = (uint8_t)(data & 0x00FF);

	return VL53L1_WriteMulti(dev, index, buffer, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	uint8_t buffer[4];

	buffer[0] = (uint8_t)(data >> 24);
	buffer[1] = (uint8_t)((data & 0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((data & 0x0000FF00) >> 8);
	buffer[3] = (uint8_t)(data & 0x0000FF);

	return VL53L1_WriteMulti(dev, index, buffer, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	uint8_t buffer[1];

	esp_err_t err = VL53L1_ReadMulti(dev, index, buffer, 1);

	*data = buffer[0];

	return err; 
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	uint8_t buffer[2];

	esp_err_t err = VL53L1_ReadMulti(dev, index, buffer, 2);

	*data = (uint16_t)(((uint16_t)buffer[0]) << 8) + (uint16_t)buffer[1];

	return err;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	uint8_t buffer[4];

	esp_err_t err = VL53L1_ReadMulti(dev, index, buffer, 4);

	*data = ((uint8_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];

	return err; 
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	vTaskDelay(pdMS_TO_TICKS(wait_ms));
	return 0;
}

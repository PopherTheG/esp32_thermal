
/*******************************************************************************
 This file is part of VL53LX Platform

 Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/



// #include <windows.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <driver/i2c.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53lx_platform.h"
#include <vl53lx_platform_log.h>

#define VL53LX_get_register_name(VL53LX_p_007,VL53LX_p_032) VL53LX_COPYSTRING(VL53LX_p_032, "");

// #include "ranging_sensor_comms.h"
// #include "power_board_defs.h"

const uint32_t _power_board_in_use = 0;


uint32_t _power_board_extended = 0;


uint8_t global_comms_type = 0;

#define  VL53LX_COMMS_CHUNK_SIZE  56
#define  VL53LX_COMMS_BUFFER_SIZE 64

#define GPIO_INTERRUPT          RS_GPIO62
#define GPIO_POWER_ENABLE       RS_GPIO60
#define GPIO_XSHUTDOWN          RS_GPIO61
#define GPIO_SPI_CHIP_SELECT    RS_GPIO51

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_PLATFORM, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#define trace_i2c(...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_NONE, \
	VL53LX_TRACE_LEVEL_NONE, VL53LX_TRACE_FUNCTION_I2C, ##__VA_ARGS__)



VL53LX_Error VL53LX_WriteMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	
	esp_err_t err;
	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_write_byte(cmd, (pdev->i2c_slave_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write(cmd, pdata, count, 1);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if(err != ESP_OK) {
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}	

	return status;
}


VL53LX_Error VL53LX_ReadMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	
	
	

	return status;
}


VL53LX_Error VL53LX_WrByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t       VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];


	buffer[0] = (uint8_t)(VL53LX_p_003);

	status = VL53LX_WriteMulti(pdev, index, buffer, 1);

	return status;
}


VL53LX_Error VL53LX_WrWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];


	buffer[0] = (uint8_t)(VL53LX_p_003 >> 8);
	buffer[1] = (uint8_t)(VL53LX_p_003 &  0x00FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

	return status;
}


VL53LX_Error VL53LX_WrDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];


	buffer[0] = (uint8_t) (VL53LX_p_003 >> 24);
	buffer[1] = (uint8_t)((VL53LX_p_003 &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((VL53LX_p_003 &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (VL53LX_p_003 &  0x000000FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);

	return status;
}


VL53LX_Error VL53LX_RdByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];

	status = VL53LX_ReadMulti(pdev, index, buffer, 1);

	*pdata = buffer[0];

	return status;
}


VL53LX_Error VL53LX_RdWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];

	status = VL53LX_ReadMulti(
					pdev,
					index,
					buffer,
					VL53LX_BYTES_PER_WORD);

	*pdata = (uint16_t)(((uint16_t)(buffer[0])<<8) + (uint16_t)buffer[1]);

	return status;
}


VL53LX_Error VL53LX_RdDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];

	status = VL53LX_ReadMulti(
					pdev,
					index,
					buffer,
					VL53LX_BYTES_PER_DWORD);

	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

	return status;
}



VL53LX_Error VL53LX_WaitUs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_us)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	float wait_ms = (float)wait_us/1000.0f;
	HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);

	SUPPRESS_UNUSED_WARNING(pdev);


	WaitForSingleObject(hEvent, (DWORD)(wait_ms + 0.5f));

	trace_i2c("WaitUs(%6d);\n", wait_us);

	return status;
}


VL53LX_Error VL53LX_WaitMs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_ms)
{
	return VL53LX_WaitUs(pdev, wait_ms * 1000);
}



VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 0;

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GetTimerFrequency: Freq : %dHz\n", *ptimer_freq_hz);
	return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count)
{
	*ptimer_count = 0;

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GetTimerValue: Freq : %dHz\n", *ptimer_count);
	return VL53LX_ERROR_NONE;
}




VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Mode((RS_GPIO_Pin)pin, (RS_GPIO_Mode)mode) != CP_STATUS_OK)
	{
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioSetMode: Status %d. Pin %d, Mode %d\n", status, pin, mode);
	return status;
}


VL53LX_Error  VL53LX_GpioSetValue(uint8_t pin, uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value((RS_GPIO_Pin)pin, value) != CP_STATUS_OK)
	{
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioSetValue: Status %d. Pin %d, Mode %d\n", status, pin, value);
	return status;

}


VL53LX_Error  VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	DWORD value = 0;

	if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Get_Value((RS_GPIO_Pin)pin, &value) != CP_STATUS_OK)
	{
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}
	else
	{
		*pvalue = (uint8_t)value;
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioGetValue: Status %d. Pin %d, Mode %d\n", status, pin, *pvalue);
	return status;
}



VL53LX_Error  VL53LX_GpioXshutdown(uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if(status == VL53LX_ERROR_NONE)
	{
		status = VL53LX_GpioSetMode((uint8_t)GPIO_XSHUTDOWN, (uint8_t)GPIO_OutputPP);
	}

	if(status == VL53LX_ERROR_NONE)
	{
		if(value)
		{
			if ((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_XSHUTDOWN, (DWORD)Pin_State_High) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
		else
		{
			if ((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_XSHUTDOWN, (DWORD)Pin_State_Low) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioXShutdown: Status %d. Value %d\n", status, value);
	return status;
}


VL53LX_Error  VL53LX_GpioCommsSelect(uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if(status == VL53LX_ERROR_NONE)
	{
		status = VL53LX_GpioSetMode((uint8_t)GPIO_SPI_CHIP_SELECT, (uint8_t)GPIO_OutputPP);
	}

	if(status == VL53LX_ERROR_NONE)
	{
		if(value)
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_SPI_CHIP_SELECT, (DWORD)Pin_State_High) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
		else
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_SPI_CHIP_SELECT, (DWORD)Pin_State_Low) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioCommsSelect: Status %d. Value %d\n", status, value);
	return status;
}


VL53LX_Error  VL53LX_GpioPowerEnable(uint8_t value)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	POWER_BOARD_CMD power_cmd;

	if(status == VL53LX_ERROR_NONE)
	{
		status = VL53LX_GpioSetMode((uint8_t)GPIO_POWER_ENABLE, (uint8_t)GPIO_OutputPP);
	}

	if(status == VL53LX_ERROR_NONE)
	{
		if(value)
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_POWER_ENABLE, (DWORD)Pin_State_High) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
		else
		{
			if((CP_STATUS)RANGING_SENSOR_COMMS_GPIO_Set_Value(GPIO_POWER_ENABLE, (DWORD)Pin_State_Low) != CP_STATUS_OK)
			{
				status = VL53LX_ERROR_CONTROL_INTERFACE;
			}
		}
	}

	if(status == VL53LX_ERROR_NONE && _power_board_in_use == 1 && value)
	{
		memset(&power_cmd, 0, sizeof(POWER_BOARD_CMD));
		power_cmd.command = ENABLE_DUT_POWER;

		if((CP_STATUS)RANGING_SENSOR_COMMS_Write_System_I2C(
			POWER_BOARD_I2C_ADDRESS, sizeof(POWER_BOARD_CMD), (uint8_t*)&power_cmd) != CP_STATUS_OK)
		{
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}
	}

	trace_print(VL53LX_TRACE_LEVEL_INFO, "VL53LX_GpioPowerEnable: Status %d. Value %d\n", status, value);
	return status;
}


VL53LX_Error  VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(function);
	SUPPRESS_UNUSED_WARNING(edge_type);

	return status;
}


VL53LX_Error  VL53LX_GpioInterruptDisable(void)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	return status;
}


VL53LX_Error VL53LX_GetTickCount(
	uint32_t *ptick_count_ms)
{



	VL53LX_Error status  = VL53LX_ERROR_NONE;

	*ptick_count_ms = timeGetTime();

	trace_print(
	VL53LX_TRACE_LEVEL_DEBUG,
	"VL53LX_GetTickCount() = %5u ms;\n",
	*ptick_count_ms);

	return status;

}


VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{


	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53LX_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	_LOG_STRING_BUFFER(register_name);

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53LX_LOG_ENABLE

	VL53LX_get_register_name(
			index,
			register_name);


	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif



	VL53LX_GetTickCount(&start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;



#ifdef VL53LX_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif
	_LOG_SET_TRACE_FUNCTIONS(VL53LX_TRACE_FUNCTION_NONE);



	while ((status == VL53LX_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53LX_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}




		VL53LX_GetTickCount(&current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}


	_LOG_SET_TRACE_FUNCTIONS(trace_functions);

	if (found == 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_TIME_OUT;

	return status;
}



#include "TCA6416A.h"
#include "TCA6416A_i2c.h"
#include "esp_err.h"

void TCA6416AInitDefault(TCA6416ARegs *Regs)
{
    Regs->Output.all = 0x0000;
    Regs->PolarityInversion.all = 0x0000;
    Regs->Config.all = 0xFFFF;
}

unsigned char TCA6416AInitI2CReg(TCA6416ARegs *Regs, uint8_t slave_addr)
{
    unsigned char return_Value = I2C_OPERATION_SUCCESSFUL;
    if (I2C_Write(2, slave_addr, TCA6416A_CONFIG_REG0, (unsigned char *)&Regs->Config, 0) != 0)
        return_Value = I2C_OPERATION_FAIL;
        
    if (I2C_Write(2, slave_addr, TCA6416A_OUTPUT_REG0, (unsigned char *)&Regs->Output, 0) != 0)
        return_Value = I2C_OPERATION_FAIL;

    if(I2C_Write(2, slave_addr, TCA6416A_POLARITY_REG0, (unsigned char *)&Regs->PolarityInversion, 0) != 0 )
        return_Value = I2C_OPERATION_FAIL;

    return return_Value;
}

void TCA6416AReadInputReg(TCA6416ARegs *Regs, uint8_t slave_addr)
{
    I2C_Read(2, slave_addr, TCA6416A_INPUT_REG0, (unsigned char *)&Regs->Input);
}

void TCA6416AReadOuputReg(TCA6416ARegs *Regs, uint8_t slave_addr)
{
    I2C_Read(2, slave_addr, TCA6416A_OUTPUT_REG0, (unsigned char *)&Regs->Output);
}

void TCA6416AWriteOutputPin(TCA6416APins Pin, uint8_t state, uint8_t slave_addr)
{
    union TCA6416A_uOutput reg_data;
    I2C_Read(2, slave_addr, TCA6416A_OUTPUT_REG0, (unsigned char *)&reg_data.all);
    if (state)
    {
        reg_data.all |= 1 << Pin;
    }
    else
    {
        reg_data.all &= ~(1 << Pin);
    }
    I2C_Write(2, slave_addr, TCA6416A_OUTPUT_REG0, (unsigned char *)&reg_data, 0);
}

void TCA6416AWriteReg(unsigned char address, unsigned char regaddress, unsigned short regVal)
{
    //I2C_Write(2, address, regaddress, (regVal), 0);
}

unsigned char TCA6416AReadReg(unsigned char address, unsigned char regaddress)
{
    return (unsigned char)I2C_Read_Byte(address, regaddress);
}

void TCA6416AWriteConfig(TCA6416ARegs *Regs, uint8_t slave_address)
{
    I2C_Write(2, slave_address, TCA6416A_CONFIG_REG0, (unsigned char *)&Regs->Config, 0);
}

void TCA6416AWriteOutput(TCA6416ARegs *Regs, uint8_t slave_address)
{
    I2C_Write(2, slave_address, TCA6416A_OUTPUT_REG0, (unsigned char *)&Regs->Output, 0);
}

void TCA6416AWritePolarity(TCA6416ARegs *Regs, uint8_t slave_address)
{
    I2C_Write(2, slave_address, TCA6416A_POLARITY_REG0, (unsigned char *)&Regs->PolarityInversion, 0);
}
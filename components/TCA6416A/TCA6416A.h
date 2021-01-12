#include <stdint.h>

/************************** I2C Address ***************************************/
#define TCA6416A_ADDRESS	    0x20 		// I2C Address 0100 00 + ADDR + R/W
											// ADDR tied to P2.2 of LaunchPad
#define TCA6416A_ADDRESS_EXPND  0x21
/************************** I2C Registers *************************************/
#define TCA6416A_INPUT_REG0		0x00		// Input status register
#define TCA6416A_INPUT_REG1		0x01		// Input status register
#define TCA6416A_OUTPUT_REG0	0x02		// Output register to change state of output BIT set to 1, output set HIGH
#define TCA6416A_OUTPUT_REG1	0x03		// Output register to change state of output BIT set to 1, output set HIGH
#define TCA6416A_POLARITY_REG0 	0x04		// Polarity inversion register. BIT '1' inverts input polarity of register 0x00
#define TCA6416A_POLARITY_REG1 	0x05		// Polarity inversion register. BIT '1' inverts input polarity of register 0x00
#define TCA6416A_CONFIG_REG0   	0x06		// Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output
#define TCA6416A_CONFIG_REG1   	0x07		// Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output
#define TCA6416A_CONFIG_INPUT   0x01
#define TCA6416A_CONFIG_OUTPUT  0x00

struct TCA6416A_sBit{
       unsigned char B0:1;
       unsigned char B1:1;
       unsigned char B2:1;
       unsigned char B3:1;
       unsigned char B4:1;
       unsigned char B5:1;
       unsigned char B6:1;
       unsigned char B7:1;
};

union TCA6416A_uInputPort{
	unsigned char all;
	struct TCA6416A_sBit bit;
};

struct TCA6416A_sInput{
	union TCA6416A_uInputPort P0;
	union TCA6416A_uInputPort P1;
};

union TCA6416A_uInput{
	unsigned short all;
	struct TCA6416A_sInput Port;
};

union TCA6416A_uOutputP0{
	unsigned char 		all;
	struct TCA6416A_sBit	bit;
};

union TCA6416A_uOutputP1{
	unsigned char 		all;
	struct TCA6416A_sBit	bit;
};

struct TCA6416A_sOutput{
	union TCA6416A_uOutputP0	P0;
	union TCA6416A_uOutputP1	P1;
};

union TCA6416A_uOutput{
	unsigned short	all;
	struct TCA6416A_sOutput	Port;
};

union TCA6416A_uPolarityInversionP0{
	unsigned char all;
	struct TCA6416A_sBit bit;
};

union  TCA6416A_uPolarityInversionP1{
	unsigned char	all;
	struct TCA6416A_sBit bit;
};

struct TCA6416A_sPolarityInversion{
	union TCA6416A_uPolarityInversionP0 P0;
	union TCA6416A_uPolarityInversionP1 P1;
};

union TCA6416A_uPolarityInversion{
	unsigned short all;
	struct TCA6416A_sPolarityInversion Port;
};

union TCA6416A_uConfigP0{
	unsigned char all;
	struct TCA6416A_sBit bit;
};

union  TCA6416A_uConfigP1{
	unsigned char	all;
	struct TCA6416A_sBit bit;
};

struct TCA6416A_sConfig{
	union TCA6416A_uConfigP0 P0;
	union TCA6416A_uConfigP1 P1;
};

union TCA6416A_uConfig{
	unsigned short all;
	struct TCA6416A_sConfig Port;
};

struct sTCA6416ARegs{
	union TCA6416A_uInput 				Input;
	union TCA6416A_uOutput 				Output;
	union TCA6416A_uPolarityInversion 	PolarityInversion;
	union TCA6416A_uConfig				Config;
};

typedef struct {
	union TCA6416A_uInput 				Input;
	union TCA6416A_uOutput				Output;
	union TCA6416A_uPolarityInversion	PolarityInversion;
	union TCA6416A_uConfig				Config;
} TCA6416ARegs;

typedef enum {
    TCA6416A_P00,
    TCA6416A_P01,
    TCA6416A_P02,
    TCA6416A_P03,
    TCA6416A_P04,
    TCA6416A_P05,
    TCA6416A_P06,
    TCA6416A_P07,
    TCA6416A_P10,
    TCA6416A_P11,
    TCA6416A_P12,
    TCA6416A_P13,
    TCA6416A_P14,
    TCA6416A_P15,
    TCA6416A_P16,
    TCA6416A_P17,
} TCA6416APins;

void TCA6416AWriteConfig(TCA6416ARegs * Regs, uint8_t slave_addr);
void TCA6416AWriteOutput(TCA6416ARegs * Regs, uint8_t slave_addr);
void TCA6416AWritePolarity(TCA6416ARegs * Regs, uint8_t slave_addr);

void TCA6416AInitDefault(TCA6416ARegs* Regs);
unsigned char TCA6416AInitI2CReg(TCA6416ARegs* Regs, uint8_t slave_addr);
void TCA6416AReadInputReg(TCA6416ARegs* Regs, uint8_t slave_addr);
void TCA6416AReadOuputReg(TCA6416ARegs* Regs, uint8_t slave_addr);
void TCA6416AWriteOutputPin(TCA6416APins Pin, unsigned char state, uint8_t slave_addr);

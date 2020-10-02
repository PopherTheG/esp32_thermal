#ifndef COMPONENTS_THERMAL_MELDIR
#define COMPONENTS_THERMAL_MELDIR

#include <stdint.h>
#include <stdlib.h>
#include "driver/spi_master.h"

// #define MDEBUG

typedef struct {
  uint8_t  SEL_SC;
  uint8_t  SEL_SCK;
  uint8_t  SEL_TCLK;
  uint8_t  ID;
  uint16_t SN;
  uint8_t  OPT_CENTX;
  uint8_t  OPT_CENTY;
  uint8_t  FAULTX[4];
  uint8_t  FAULTY[4];
  uint8_t  THER_OFFSET;
} sensor_into_reg_t;

typedef struct {
  uint8_t  FRAME_FLG;
  uint8_t  SHT_FLG;
  uint8_t  WUP;
  uint8_t  WUP_RD;
  uint8_t  ACT_SHT;
  uint8_t  ACT_SHT_RD;
  uint16_t TEMP_BRD;
  uint8_t  CRC8[32];
  uint8_t  SEL_LINE;
  uint8_t  SEL_CLST;
  uint8_t  REG_RD;
  uint8_t  P[4];
  uint16_t PIXEL[4];
} frame_reg_t;

typedef struct {
  uint16_t pixel[32][80];
  uint16_t brd;
} thermal_img_t;

#define SUCCESS  0
#define ERROR    1
#define PWROFF   0
#define PWRON    1

#define ADD_FRAMEREG       0x5A
#define ADD_SENSORINFOREG  0x75

int WakeUP(spi_device_handle_t spi);
int INIT_sensor_sc(sensor_into_reg_t *sensInfo, uint8_t sc);
int ShutterAct(frame_reg_t *frame);
int GetThermalImage(frame_reg_t *frame, thermal_img_t *thermalImg);
#ifdef MDEBUG
#define PRINTF(...)  printf(__VA_ARGS__); 
#else
#define PRINTF(...)
#endif

#endif /* COMPONENTS_THERMAL_MELDIR */

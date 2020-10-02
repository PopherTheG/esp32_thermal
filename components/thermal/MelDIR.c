#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "MelDIR.h"

#define TAG     "melDIR"

spi_device_handle_t spi_handle;

static uint8_t CRC8tbl[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

void TimerWait(uint32_t usec)
{
  vTaskDelay(pdMS_TO_TICKS(usec / 1000) + 2);
}

static void sensor_reg_2_tx(sensor_into_reg_t *reg, uint8_t *txdata) {
    int i;
    for (i = 0; i < 0x30; i++) {
        txdata[i] = 0;
    }

    txdata[0x00] = ADD_SENSORINFOREG;
    txdata[0x03] = ( (reg->SEL_SC & 0x1 ) << 6 );
    txdata[0x04] = reg->SEL_SCK;
    txdata[0x05] = 0x19;
    txdata[0x06] = 0x00;
    txdata[0x07] = 0x80;
    txdata[0x0F] = 0x80;
    txdata[0x13] = 0x00;
}

static int crcCheck( uint16_t *pdata, uint8_t crc )
{
  uint8_t calc;
  int i;

  calc = 0;
  for(i=0; i<80; i++) {
    calc = CRC8tbl[calc ^ ((pdata[i]&0xff00)>>8)];
    calc = CRC8tbl[calc ^  (pdata[i]&0x00ff)    ];
  }

  if(calc == crc){
    return SUCCESS;
  } else {
    return ERROR;
  }
}

static int parityCheck(uint16_t pixel, uint8_t p) { 
    uint16_t val = 0;

    val = pixel;
    val ^= (val >> 16);
    val ^= (val >>  8);
    val ^= (val >>  4);
    val ^= (val >>  2);
    val ^= (val >>  1);

    if( (~val & 0x1) == p ){
        return SUCCESS;
    } else {
        return ERROR;
    }
}

static void sensor_rx_2_reg(sensor_into_reg_t *reg, uint8_t *rxdata) {
    int i;

    reg->SEL_SC      = ( (rxdata[0x04] & 0x40) >> 6 );
    reg->SEL_TCLK    = rxdata[0x05] & 0x7F;
    reg->ID          = rxdata[0x01];
    reg->SN          = ( (rxdata[0x02] & 0x07) << 8 ) | (rxdata[0x03] & 0xFF);
    reg->OPT_CENTX   = rxdata[0x08] & 0x7F;
    reg->OPT_CENTY   = rxdata[0x09] & 0x7F;

    for( i=0; i<4; i++ ){
        reg->FAULTX[i] = rxdata[0x0A+(i*2)] & 0x1F;
        reg->FAULTY[i] = rxdata[0x0B+(i*2)] & 0x7F;
    }

    reg->THER_OFFSET = rxdata[0x12];
}

static void spi_trans(uint8_t *tx_data, uint8_t *rx_data, uint32_t count) {
    esp_err_t ret = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = count * 8;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    // transaction.flags = SPI_TRANS_USE_RXDATA;
    ret = spi_device_polling_transmit(spi_handle, &t);
    // ret = spi_device_transmit(spi_handle, &transaction);
    
    if (ret != ESP_OK) {        
        ESP_LOGE(TAG, "spi_trans error");
    }
}

static void frame_reg_2_tx(frame_reg_t *reg, uint8_t *txdata) 
{
    int i;
    for (i = 0; i < 0x30; i++) {
        txdata[i] = 0;        
    }

    txdata[0x00] = ADD_FRAMEREG;
    txdata[0x00] = ADD_FRAMEREG;
    txdata[0x01] = 0x14
                | ( (reg->WUP      & 0x1 ) << 6 )
                | ( (reg->ACT_SHT  & 0x1 ) << 1 );
    txdata[0x06] = ( (reg->SEL_LINE & 0x1F) << 1 );
    txdata[0x07] = 0x80
                | ( (reg->SEL_CLST & 0x1F) << 1 )
                | ( (reg->REG_RD   & 0x1 ) << 0 );
    txdata[0x0F] = 0x80;
    txdata[0x17] = 0x80;
    txdata[0x1F] = 0x80;
    txdata[0x27] = 0x80;
    txdata[0x2E] = ( (reg->SEL_LINE & 0x1F) << 1 );
    txdata[0x2F] = 0x80
                | ( (reg->SEL_CLST & 0x1F) << 1 )
                | ( (reg->REG_RD   & 0x1 ) << 0 );

    PRINTF(" Frame Write: >>>");
    for (i = 0; i < 0x2f + 1; i++)
    {
        PRINTF("0x%02X ", txdata[i]);
    }
    PRINTF("\n");
}

static void frame_rx_2_reg(frame_reg_t *reg, uint8_t *rxdata) {
    int i;
    reg->FRAME_FLG  = (rxdata[0x00] & 0x08) >> 3;
    reg->SHT_FLG    = (rxdata[0x00] & 0x04) >> 2;
    reg->WUP        = (rxdata[0x02] & 0x40) >> 6;
    reg->WUP_RD     = (rxdata[0x03] & 0x40) >> 6;
    reg->ACT_SHT_RD = (rxdata[0x02] & 0x02) >> 1;
    reg->TEMP_BRD   = ( (rxdata[0x06] & 0x03) << 8 ) | rxdata[0x07];

    for( i=0; i<32; i++ ){
        reg->CRC8[i] = rxdata[0x08+i];
    }

    for( i=0; i<4; i++ ){
        reg->P[i]     = (rxdata[0x29+(i*2)] & 0x40) >> 6;
        reg->PIXEL[i] = ( (rxdata[0x29+(i*2)] & 0x3F) << 8) | rxdata[0x28+(i*2)];
    }

    PRINTF("Frame Read: <<< ");
    for (i = 0; i < 0x30; i++)
    {
        PRINTF("0x%02X ", rxdata[i]);
    }
    PRINTF("\n");
}

static void printSensorInfoReg(sensor_into_reg_t *sensInfo) {
    int i;

    PRINTF("SENSOR INFO.\n");
    PRINTF("SEL_SC      = 0x%02x\n", sensInfo->SEL_SC);
    PRINTF("SEL_SCK     = 0x%02x\n", sensInfo->SEL_SCK);
    PRINTF("SEL_TCLK    = 0x%02x\n", sensInfo->SEL_TCLK);
    PRINTF("ID          = 0x%02x\n", sensInfo->ID);
    PRINTF("SN          = 0x%03x\n", sensInfo->SN);
    PRINTF("OPT_CENTX   = 0x%02x\n", sensInfo->OPT_CENTX);
    PRINTF("OPT_CENTY   = 0x%02x\n", sensInfo->OPT_CENTY);
    for (i = 0; i < 4; i++)
    {
        PRINTF("FAULTX[%d]   = 0x%02x\n", i, sensInfo->FAULTX[i]);
        PRINTF("FAULTY[%d]   = 0x%02x\n", i, sensInfo->FAULTY[i]);
    }
    PRINTF("THER_OFFSET = 0x%02x\n", sensInfo->THER_OFFSET);
}

int WakeUP(spi_device_handle_t spi) {
    ESP_LOGI( TAG, "Wake up!" );
    spi_handle = spi;
    frame_reg_t frame_reg;

    uint8_t TxData[48];
    uint8_t RxData[48];
    uint8_t cnt;

    frame_reg.WUP       = 1;
    frame_reg.ACT_SHT   = 0;
    frame_reg.SEL_LINE  = 0;
    frame_reg.SEL_CLST  = 0;
    frame_reg.REG_RD    = 0;
    frame_reg_2_tx(&frame_reg, TxData);

    spi_trans(TxData, RxData, 8);
    TimerWait(250000); // wait 250ms
    spi_trans(TxData, RxData, 8);

    frame_rx_2_reg(&frame_reg, RxData);
    ESP_LOGI(TAG, "Wake up %d", frame_reg.WUP_RD  );

    if (frame_reg.WUP_RD == 1)
    {
        PRINTF("   SUCCESS.\n");
        return SUCCESS;
    }

    PRINTF("Wake up retry...\n");
    for (cnt = 0; cnt < 3; cnt++)
    {
        PRINTF("%d ", cnt + 1);
        TimerWait(250000); // wait 250ms
        spi_trans(TxData, RxData, 8);
        frame_rx_2_reg(&frame_reg, RxData);
        ESP_LOGI(TAG, "Wake up %d", frame_reg.WUP_RD  );
        if (frame_reg.WUP_RD == 1)
        {
            PRINTF("   SUCCESS.\n");
            return SUCCESS;
        }
    }

    PRINTF("   ERROR.\n");
    return ERROR;
}

int INIT_sensor_sc( sensor_into_reg_t *sensInfo, uint8_t sc) {
    uint8_t tx_data[48];
    uint8_t rx_data[48];

    sensInfo->SEL_SC = sc;
    sensInfo->SEL_SCK = 48000000u / 1000000u;

    sensor_reg_2_tx(sensInfo, tx_data);
    printSensorInfoReg(sensInfo);
    spi_trans(tx_data, rx_data, 0x14);
    sensor_rx_2_reg(sensInfo, rx_data);
    printSensorInfoReg(sensInfo);

    return SUCCESS;
}

int ShutterAct(frame_reg_t *frame) {
    ESP_LOGI(TAG, "Shutter Calibration");
    uint8_t TxData[48];
    uint8_t RxData[48];
    uint8_t cnt;

    frame->WUP      = 1;
    frame->ACT_SHT  = 1;
    frame->SEL_LINE = 0;
    frame->SEL_CLST = 0;
    frame->REG_RD   = 0;
    frame_reg_2_tx( frame, TxData );
    spi_trans( TxData, RxData, 0x8 );

    frame->ACT_SHT  = 0;
    frame_reg_2_tx( frame, TxData );
    spi_trans( TxData, RxData, 0x8 );

    TimerWait( 4800000u );  // 4.8s

    spi_trans( TxData, RxData, 0x8 );
    frame_rx_2_reg( frame, RxData );
    if( frame->SHT_FLG == 1 ){
        PRINTF( "   SUCCESS.\n" );
        return SUCCESS;
    }

    PRINTF( "\nShutterACT retry..." );
    for( cnt=0; cnt<3; cnt ++ ){
        PRINTF( "%d ", cnt+1 );
        TimerWait( 4800000u );  // 4.8s
        spi_trans( TxData, RxData, 0x8 );
        frame_rx_2_reg( frame, RxData );
            if( frame->SHT_FLG == 1 ){
        PRINTF("   SUCCESS.\n");
        ESP_LOGI(TAG,"  SUCCESS");
        return SUCCESS;
        }
    }

    ESP_LOGE(TAG, "   ERROR");
    PRINTF( "   ERROR.\n" );
    return ERROR;
}

int GetThermalImage(frame_reg_t *frame, thermal_img_t *img) {    
    uint8_t TxData[48];
    uint8_t RxData[48];
    uint8_t line, clst, idx;
    uint8_t rdLine = 0, rdClst = 0;
    int RETVAL = SUCCESS;

    frame->WUP_RD   = 1;
    frame->ACT_SHT  = 0;
    frame->SEL_LINE = 0;
    frame->SEL_CLST = 0;
    frame->REG_RD   = 0;
    frame_reg_2_tx(frame, TxData);

    do {
        spi_trans(TxData, RxData, 0x8);
        frame_rx_2_reg(frame, RxData);
    } while (frame->FRAME_FLG != 1);

    frame->REG_RD = 1;
    for ( line = 0; line < 32; line++ ) 
    {
        for ( clst = 0; clst < 20; clst++ ) 
        {
            frame->SEL_LINE = line;
            frame->SEL_CLST = clst;

            if(line == 0 && clst == 0) {
                frame_reg_2_tx(frame, TxData);
                spi_trans(TxData, RxData, 0x28);
                frame_rx_2_reg(frame, RxData);

                rdLine = line;
                rdClst = clst;
                continue;            
            }

            frame_reg_2_tx(frame, TxData);
            spi_trans(&TxData[0x28], &RxData[0x28], 0x2F-0x28+1);
            frame_rx_2_reg(frame, RxData);

            for ( idx = 0; idx < 4; idx++ ) {
                img->pixel[rdLine][rdClst*4+idx] = frame->PIXEL[idx];
                if(parityCheck(frame->PIXEL[idx], frame->P[idx]) == ERROR) {
                    PRINTF("Parity Error in line=%d, clst=%d, idx=%d\n", rdLine, rdClst, idx);
                    RETVAL = ERROR;
                }
            }

            if(line == 31 && clst == 19) {
                frame->REG_RD = 0;
                frame_reg_2_tx(frame, TxData);
                spi_trans(&TxData[0x28], &RxData[0x28], 0x2F-0x28+1);
                frame_rx_2_reg(frame, RxData);
                for (idx=0; idx<4; idx++) {
                    img->pixel[line][clst*4+idx] = frame->PIXEL[idx];
                    if( parityCheck( frame->PIXEL[idx], frame->P[idx] ) == ERROR ){
                        PRINTF( "Parity Error in line=%d, clst=%d, idx=%d\n", line, clst, idx );
                        RETVAL = ERROR;
                    }
                }
            }

            if(rdClst == 19)
            if(crcCheck(img->pixel[rdLine], frame->CRC8[rdLine]) == ERROR) {
                PRINTF("CRC Error in line=%d\n", rdLine);
                RETVAL = ERROR;
            }

            rdLine = line;
            rdClst = clst;
        }
    }

    img->brd = frame->TEMP_BRD;

    PRINTF(" TEMP_BRD = %d\n", frame->TEMP_BRD);

    if(RETVAL == SUCCESS) {
        PRINTF("    SUCCESS.\n");
    } else {
        PRINTF("    ERROR.\n");
    }

    return RETVAL;
}
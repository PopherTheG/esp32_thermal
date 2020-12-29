
#include "TCA6416A.h"
#include "io.h"

static TCA6416ARegs regs;

void io_init(void) {
    TCA6416AInitDefault(&regs);

    

    TCA6416AInitI2CReg(&regs, TCA6416A_ADDRESS_EXPND);
}

void io_set_red_led(uint8_t state) {

}

void io_set_blue_led(uint8_t state) {

}

void io_set_green_led(uint8_t state) {

}

void io_set_buzzer(uint8_t state) {

}

void io_set_relay(uint8_t level) {

}

uint8_t io_get_relay(void) {

}
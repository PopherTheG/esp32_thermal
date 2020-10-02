#ifndef COMPONENTS_THERMAL_THERMAL_SPI
#define COMPONENTS_THERMAL_THERMAL_SPI

#include <stdlib.h>
#include "MelDIR.h"

typedef enum {
    IR_EVENT_WAKEUP_SUCCESS,
    IR_EVENT_WAKEUP_FAIL,
    IR_EVENT_INIT_FAIL,
    IR_EVENT_INIT_SUCCES,
    IR_EVENT_SHUTTER_SUCCES,
    IR_EVENT_SHUTTER_FAIL,
    IR_EVENT_THERMAL_IMG_FAIL,
    IR_EVENT_THERMAL_IMG_SUCCES,
};

typedef struct {
    uint8_t id;    
} ir_event_t;

typedef struct {
  float Tpixel[32][80];
  float Tbrd;
  float max_temp;
} temp_img_t;

typedef void (*ir_app_cb)(ir_event_t* event);

int init_thermal(ir_app_cb event_handler);

void thermal_start(void);

void thermal_stop(void);

void thermal_app_deinit(void);

void thermal_get_data(thermal_img_t* data);

#endif /* COMPONENTS_THERMAL_THERMAL_SPI */

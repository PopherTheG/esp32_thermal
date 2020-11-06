#ifndef COMPONENTS_VL53L3CX_VL53L3CX
#define COMPONENTS_VL53L3CX_VL53L3CX

#include "vl53lx_error_codes.h"

#define VL53LX_LOG_ENABLE   1

VL53LX_Error init_vl53l3cx(void);

void vl53Ll3cx_start_app(void);

void vl53l3cx_stop_app(void);

#endif /* COMPONENTS_VL53L3CX_VL53L3CX */

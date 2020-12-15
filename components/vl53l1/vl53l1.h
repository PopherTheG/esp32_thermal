#ifndef COMPONENTS_VL53L1_VL53L1
#define COMPONENTS_VL53L1_VL53L1

#include <stdint.h>
#include <stdlib.h>

#define NOBODY 0
#define SOMEONE 1
#define LEFT 0
#define RIGHT 1

#define DIST_THRESHOLD_MAX 500
// #define PEOPLE_COUNTING_MODE

typedef enum {
    EVT_THRESHOLD_INSIDE,
    EVT_THRESHOLD_OUTSIDE,
} vl53l1_event_id_t;

typedef struct vl53l1_event {
    vl53l1_event_id_t id;
    uint16_t distance;
} vl53l1_event_t;

typedef void (*vl53l1_app_cb)(vl53l1_event_t *evt);

uint8_t vl53l1_init(vl53l1_app_cb app_cb);

void vl53l1_deinit(void);

void vl53l1_start_app(void);

void vl53l1_stop_app(void);

#endif /* COMPONENTS_VL53L1_VL53L1 */

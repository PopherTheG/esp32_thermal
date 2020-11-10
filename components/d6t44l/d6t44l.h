#ifndef COMPONENTS_D6T44L_D6T44L
#define COMPONENTS_D6T44L_D6T44L

#define D6T_I2C_PORT    0
#define D6T_ADDR        0x0A
#define D6T_CMD         0x4C

#define N_ROW           4
#define N_PIXEL         (4 * 4)
#define N_READ          ((N_PIXEL + 1) * 2 + 1)

#define WRITE_ADDR      ((D6T_ADDR << 1) | I2C_MASTER_WRITE)
#define READ_ADDR       ((D6T_ADDR << 1) | I2C_MASTER_READ)
#define CALIBRATE
#define SAMPLING        10
// #define DEBUG

#define TEMPERATURE_THRESHOLD   37.4

#define LED_BLUE    16
#define LED_GREEN   33
#define LED_RED     25
#define BUZZER_IO   17

typedef enum {
    TEMP_EVT_DATA_READY,
    TEMP_EVT_ERROR,
} d6t44l_event_id_t;

typedef struct d6t44l_event {
    d6t44l_event_id_t id;    
} d6t44l_event_t;

typedef void (*d6t44l_app_cb)(d6t44l_event_t *evt);

int D6T44l_init(d6t44l_app_cb app_cb);

void D6T44_app_run(void);

void D6T44_app_stop(void);

void D6T44_update_temp(void);

void D6T44_reset(void);

#endif /* COMPONENTS_D6T44L_D6T44L */
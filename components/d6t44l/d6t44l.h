#ifndef COMPONENTS_D6T44L_D6T44L
#define COMPONENTS_D6T44L_D6T44L

#define D6T_I2C_PORT    0
#define D6T_ADDR    0x0A
#define D6T_CMD     0x4C

#define N_ROW       4
#define N_PIXEL     (4 * 4)
#define N_READ  ((N_PIXEL + 1) * 2 + 1)

int D6T44l_init(void);

void app_run(void);

#endif /* COMPONENTS_D6T44L_D6T44L */
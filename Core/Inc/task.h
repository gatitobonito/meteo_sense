#include "stm32f1xx_hal.h"

#define HUMIDITY_K										((float)125)
#define HUMIDITY_B										((float)-6)



void init(void);
void main_task(void);
void power_off(void);
void set_sense_si7007(uint32_t period, uint32_t pulse);
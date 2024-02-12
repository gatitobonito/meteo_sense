/**
  ******************************************************************************
  * @file    pwm_task.h
  * @author  
  * @version V0.0.1
  * @date    02-Feb-2024
  * @brief   TIM init and pwm tasks
  ******************************************************************************
  */


#define TIM3_PWM_PERIOD		 18000
#define TIM3_PWM_PULSE     1080
#define PWM_PULSE_OPEN_CLOSE_DELTA 960
#define PWM_PERIOD_DELTA 3000
#define PWM_PULSE_DELTA 500


void pwm_init(void);
void pwm_alarm_init(void);
// void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void sense_si7007_init(void);
#include "stm32f1xx.h"
#include "pwm_task.h"
#include "stm32f1xx_hal_tim.h"
#include "main.h"
// #include "pwm_task.h"

TIM_HandleTypeDef htim2 = {0};
volatile uint32_t period = 0; 
volatile uint32_t pulse = 0;

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15 ;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30000; 
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);


}

static  void pwm_capture_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Настраиваем ножку на вход таймера
    GPIO_InitTypeDef sGPIO;
    sGPIO.Speed = GPIO_SPEED_FREQ_HIGH;
    sGPIO.Pull = GPIO_NOPULL;
    sGPIO.Mode = GPIO_MODE_AF_INPUT;
    sGPIO.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &sGPIO);

    // Настраиваем таймер на захват (TIM2, CH1), разрешение 1 мкс
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 15;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_DISABLE;
    htim2.Init.RepetitionCounter = 0x00;
    HAL_TIM_IC_Init(&htim2);

    TIM_MasterConfigTypeDef sMaster;
    sMaster.MasterOutputTrigger = TIM_TRGO_RESET;
    sMaster.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMaster);

    TIM_IC_InitTypeDef sIC;
    sIC.ICPolarity = TIM_ICPOLARITY_RISING;
    sIC.ICFilter = 0;
    sIC.ICPrescaler = TIM_ICPSC_DIV1;
    sIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim2, &sIC, TIM_CHANNEL_2);

    sIC.ICPolarity = TIM_ICPOLARITY_FALLING;
    sIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim2, &sIC, TIM_CHANNEL_1);

    NVIC_EnableIRQ(TIM2_IRQn);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}


void sense_si7007_init(void)
{
  //TIM2, Channel2, PA1
    MX_TIM2_Init();
    pwm_capture_init();
}




void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // колбек по захвату
{
        if(htim->Instance == TIM2)
        {
                if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
                {
                        TIM2->CNT=0;
                        period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
                        pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); 
                }
        }
}


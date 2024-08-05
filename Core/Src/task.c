#include "task.h"
#include "stdint.h"
#include "lora_sx1276.h"
extern SPI_HandleTypeDef hspi2;
static lora_sx1276 lora;
float humidity;
static senses_t senses = {0,0,0};
extern uint32_t period; 
extern uint32_t pulse;
static void led_update_task(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(100);
}
static void si7007_task()
{

}

static uint8_t spiXmit (SPI_TypeDef *SPIx, uint8_t byte)
{
	SPIx->DR = byte;
	while (!(SPIx->SR & SPI_SR_RXNE));
	return SPIx->DR;
}

static uint16_t readValue (void)
{
	uint16_t temp=0;
	uint16_t temp2=0;
    uint8_t data=0x00;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, &data, &temp, 1, 100);
	temp = temp << 8;
	HAL_SPI_TransmitReceive(&hspi2, &data, &temp2, 1, 100);
	temp |= temp2;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	return temp;
}

void main_task(void)
{

    led_update_task();
    // lps331_task();
    ad7814_task();
    read_sense_si7007(period, pulse);
    HAL_Delay(100);
}


void power_off(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0}; 
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(100); 
}


void read_sense_si7007(uint32_t period, uint32_t pulse)
{
    float temp;
	temp = (float)pulse / (float)period;
	senses.humidity = HUMIDITY_B + HUMIDITY_K * temp;
}

uint8_t reciever_init(SPI_HandleTypeDef *hspi)
{
    SPI_HandleTypeDef hspi1;
    
    uint8_t res = lora_init(&lora, &hspi, GPIOA, GPIO_PIN_4, LORA_CUSTOM_FREQUENCY_US);

    return res;
}

uint8_t lora_send(uint8_t *data, uint8_t length)
{
    //amp on for trasnmitt
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
    uint8_t res = lora_send_packet(&lora, data, length);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    return res;
}

uint8_t lora_recieve(void)
{
    // Receive buffer
    uint8_t buffer[32];
    uint8_t res;
    // Put LoRa modem into continuous receive mode
    lora_mode_receive_continuous(&lora);
    // Wait for packet up to 10sec

    uint8_t len = lora_receive_packet_blocking(&lora, buffer, sizeof(buffer), 10000, &res);


    return res;
}

void ad7814_task(void)
{
	int16_t reg;
	
	reg = readValue ();
	//reset dump data
	reg >>= 5;
	
	// 10 bit sense with 9 bit is sign
	if (reg & 0x0200) 
    {
        reg |= 0xfc00;
    }
	// result in Celsius
	senses.temperature = (float)reg * KOEF_CELSIUS;	
}



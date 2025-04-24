
#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>

uint8_t rxData[32];
uint8_t rxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};


int receiver(void)
{
    HAL_Init(); // Reset all peripherals
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef initStrC = 
    {
        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_LOW,
        GPIO_NOPULL
    };

    HAL_GPIO_Init(GPIOC, &initStrC);

    setupSPI();

    nrfRxMode(rxAddress, 10);

    while(1)
    {
        HAL_Delay(1000);
        if(isDataAvailable(1) == 1)
        {
            receiveData(rxData);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
        }
        else
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); 
        }
        
    }
}
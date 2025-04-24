#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>


// data array to be sent
uint8_t txData[NRF24L01P_PAYLOAD_LENGTH] = "worl";
uint8_t txAddress[] = {0xEE, 0xEE, 0xEE, 0xEE, 0xEE};

int sender(void)
{
    HAL_Init(); // Reset all peripherals
    SystemClock_Config(); //Configure the system clock

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

    nrf24Init();
    nrf24TxMode(txAddress, 10);

    while (1)
    {   
        HAL_Delay(750);
        if(transmitData(txData) == 1)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
        }
        else
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); 
        }

    }
    //gyroscope();
    return 1;
}
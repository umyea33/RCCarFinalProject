#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>


// data array to be sent
uint8_t txData[] = "Hello World\n";
uint8_t txAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

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
    // txData[0] = 'L';
    // txData[1] = 150;
    // txData[2] = 'F';
    // txData[3] = 50;
    while (1)
    {   
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
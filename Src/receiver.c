
#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>

uint8_t rxData[NRF24L01P_PAYLOAD_LENGTH];
uint8_t rxAddress[] = {0xEE, 0xEE, 0xEE, 0xEE, 0xEE};

int receiver(void)
{
    HAL_Init();
    //motor();
    ultrasonic_gpio_init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef initStrC = 
    {
        GPIO_PIN_8 | GPIO_PIN_9,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_LOW,
        GPIO_NOPULL
    };

    HAL_GPIO_Init(GPIOC, &initStrC);

    setupSPI();

    nrf24Init();
    nrfRxMode(rxAddress, 10);

    while(1)
    {
        // int data = ultrasonic_get_distance_cm();

        // if(data < 20)
        //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

        if(isDataAvailable(1) == 1)
        {
            receiveData(rxData);
            //setDirection(rxData);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
        }
        else
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        }
        
    }
}
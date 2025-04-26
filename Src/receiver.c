
#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>

uint8_t rxData[NRF24L01P_PAYLOAD_LENGTH];
uint8_t rxAddress[] = {0xEE, 0xEE, 0xEE, 0xEE, 0xEE};

// M1 (Right) In1 is backwards In2 is forward
// M2 (Left)  In4 is backwards In3 is forward
// PC10 = ENA  PC11 = ENB
// PC8 = In1, PC9 = In2, PC3 = In3, and PC4 = In4

int receiver(void)
{
    HAL_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    ultrasonic_gpio_init();

    GPIO_InitTypeDef initStrC = 
    {
        GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_3 | GPIO_PIN_4,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_LOW,
        GPIO_NOPULL
    };

    HAL_GPIO_Init(GPIOC, &initStrC);

    setupSPI();

    nrf24Init();
    nrfRxMode(rxAddress, 10);

    initMotorPins();

    while(1)
    {
        // int data = ultrasonic_get_distance_cm();

        // if(data < 20)
        //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
        if(isDataAvailable(1) == 1)
        {
            receiveData(rxData);
            processCommand(rxData[0]);

            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
        }
        else
        {
            // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        }
        
    }
}

void processCommand(char c)
{
    if(c == 'n')
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    else if(c == 'f')
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    else if(c == 'b')
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    }
    else if(c == 'l')
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    }
    else if(c == 'r')
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    }
}

void initMotorPins()
{
    // Set the two enables to high
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);

    // Set all the inputs to low
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}
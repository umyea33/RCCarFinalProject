#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>


// data array to be sent
uint8_t txData[NRF24L01P_PAYLOAD_LENGTH] = {0, 0, 0, 0};

// for rx interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

int sender(void)
{
    HAL_Init(); // Reset all peripherals
    SystemClock_Config(); //Configure the system clock

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef initStrC = 
    {
        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_13,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_LOW,
        GPIO_NOPULL
    };


    HAL_GPIO_Init(GPIOC, &initStrC);

    setupSPI();

    nrf24l01p_tx_init(2500, _1Mbps);

    while (1)
    {
        // change tx datas
        for(int i = 0; i < NRF24L01P_PAYLOAD_LENGTH; i++)
            txData[i] = i + 1;

        // transmit
        nrf24l01p_tx_transmit(txData);
        HAL_Delay(1000);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    }
    //gyroscope();


    return 1;
}

#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>

uint8_t* rxData[NRF24L01P_PAYLOAD_LENGTH] = {0, 0, 0, 0};
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
		nrf24l01p_rx_receive(rxData); // read data when data ready flag is set
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

}

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

    nrf24l01p_rx_init(2500, _1Mbps);

    while(1)
    {
        // Process data
        if(rxData[0] == 1)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

            // Clear data
            rxData[0] = 0;
            rxData[1] = 0;
            rxData[2] = 0;
            rxData[3] = 0;
        }
    }
    return 1;
}
#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>


// data array to be sent
uint8_t tData[NRF24L01P_PAYLOAD_LENGTH] = {0, 0, 0, 0};
uint8_t *txData = tData;

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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

    setupSPI();
    irq_pin_init();

    nrf24l01p_tx_init(2402, _1Mbps);

    txData[0] = 8;
    txData[1] = 6;
    txData[2] = 4;
    txData[3] = 2;

    nrf24l01p_tx_transmit(txData);

    // while (1)
    // {
    //     // change tx datas
    //     for(int i = 0; i < NRF24L01P_PAYLOAD_LENGTH; i++)
    //         txData[i] = 3;

    //     // transmit
    //     nrf24l01p_tx_transmit(txData);
    //     HAL_Delay(1000);
    //     nrf24l01p_tx_irq(); // Check status and clear IRQ
        
    //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    // }
    //gyroscope();


    return 1;
}
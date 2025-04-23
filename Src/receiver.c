
#include "main.h"
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>

uint8_t rData[NRF24L01P_PAYLOAD_LENGTH] = {0, 0, 0, 0};
uint8_t *rxData = rData;

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
		nrf24l01p_rx_receive(rxData); // read data when data ready flag is set
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
}

void EXTI4_15_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

        uint8_t status = nrf24l01p_get_status();

        if (status & 0x40) { // RX_DR
            uint8_t buffer[NRF24L01P_PAYLOAD_LENGTH];
            nrf24l01p_rx_receive(buffer);
            if(rxData[0] == 3)
            {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

                // Clear data
                rxData[0] = 0;
                rxData[1] = 0;
                rxData[2] = 0;
                rxData[3] = 0;
            }
        }

        if (status & 0x20) {
            nrf24l01p_clear_tx_ds(); // TX done
        }

        if (status & 0x10) {
            nrf24l01p_clear_max_rt(); // Max retransmit
        }
    }
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

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1); // Green

    setupSPI();
    irq_pin_init();

    nrf24l01p_rx_init(2402, _1Mbps);

    // While data is not ready then do nothing.
    volatile uint8_t reg = read_register(NRF24L01P_REG_FIFO_STATUS);
    uint8_t isEmpty = reg & 0x1;
    while(isEmpty)
    {
        reg = read_register(NRF24L01P_REG_FIFO_STATUS);
        isEmpty= reg & 0x1;
    }
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);

    // Now something must be in rxData
    nrf24l01p_rx_receive(rxData); // Will also clear RX_DR

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // Orange

    if(rxData[0] == 0)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
    else if(rxData[0] == 8)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
    else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
    // HAL_Delay(1000);

    // nrf24l01p_rx_receive(rxData); // Will also clear RX_DR

    // if(rxData[0] == 0)
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
    // else if(rxData[0] == 8)
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
    // else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

    // while(1)
    // {
    //     HAL_Delay(1000);
    //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

    //     // You can poll status or check IRQ pin
    //     if (nrf24l01p_get_status() & 0x40) // RX_DR bit
    //     {
    //         nrf24l01p_rx_receive(rxData); // Will also clear RX_DR

    //         HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

    //         // Process data
    //         if(rxData[0] == 3)
    //         {
    //             HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

    //             // Clear data
    //             rxData[0] = 0;
    //             rxData[1] = 0;
    //             rxData[2] = 0;
    //             rxData[3] = 0;
    //         }
    //     }
        
        
    // }
    // return 1;
}
#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>


void motor(void)
{
    DAC_Init();
    Motor_GPIO_Init();
}

void DAC_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR |= DAC_CR_EN1;
}

void Motor_GPIO_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // PA4 - DAC out (already configured in DAC)
    // PA5, PA6 - Motor direction
    GPIOA->MODER |= (1 << (5 * 2)) | (1 << (6 * 2));
    GPIOA->OTYPER &= ~((1 << 5) | (1 << 6));
}

void Motor_SetDirection(int forward) {
    if (forward) {
        GPIOA->ODR |= (1 << 5);
        GPIOA->ODR &= ~(1 << 6);
    } else {
        GPIOA->ODR &= ~(1 << 5);
        GPIOA->ODR |= (1 << 6);
    }
}

void SetRPM(uint8_t duty) {
    if (duty > 100) duty = 100;
    DAC->DHR8R1 = (255 * duty) / 100;
}

void setDirection(uint8_t* data)
{
    if(data[0] == 1) // Left
    {
        Motor_SetDirection(0);
        SetRPM(90);
    }
    else if(data[1] == 1) // Forward
    {
        Motor_SetDirection(1);
        SetRPM(90);
    }
    else if(data[2] == 1) // Right
    {
        Motor_SetDirection(1);
        SetRPM(90);
    }
    else if(data[3] == 1) // Back
    {
        Motor_SetDirection(0);
        SetRPM(90);
    }
    else
    {
        SetRPM(0);
        GPIOA->ODR &= ~((1 << 5) | (1 << 6));
    }
}
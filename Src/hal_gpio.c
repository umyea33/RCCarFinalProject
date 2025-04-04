#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void My_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    // Initialize LED pins
    // PC6 = Red LED
    // PC7 = Blue LED
    // PC8 = Oragne LED
    // PC9 = Green LED
    if(GPIOx == GPIOC)
    {
        // Sets the four LEDs to General Purpose Output mode
        GPIOx->MODER |= 0x00055000;
        GPIOx->MODER &= ~(0x000AA000);

        // Sets the four LEDs to Push Pull Output type
        GPIOx->OTYPER |= 0x00000000;
        GPIOx->OTYPER &= ~(0x000003C0);

        // Sets the four LEDs to Low Speed
        GPIOx->OSPEEDR |= 0x00000000;
        GPIOx->OSPEEDR &= ~(0x00055000);

        // Sets the four LEDs to No Pull-Up or Pull-Down resistors
        GPIOx->PUPDR |= 0x00000000;
        GPIOx->PUPDR &= ~(0x000FF000);
    }

    // Initialize Button
    // PA0 = USER Button
    if(GPIOx == GPIOA)
    {
        // Sets the User Button (PA0) to Digital Input mode
        GPIOx->MODER |= 0x00000000;  // Set  not needed cuz nothing is set
        GPIOx->MODER &= ~(0x00000003);  // Clear the bits that should be zero
        
        // Sets the User Button (PA0) to Low Speed
        GPIOx->OSPEEDR |= 0x00000000; // Set  not needed cuz nothing is set
        GPIOx->OSPEEDR &= ~((1) | (1 << 1)); // Clear  0x00000003
        
        // Sets the User Button (PA0) to a Pull-Down resistor
        GPIOx->PUPDR |= ~(1 << 1);  // Set 0x000000002
        GPIOx->PUPDR &= ~(1); // Clear 0x00000001
    }
}

/*
void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
}
*/

GPIO_PinState My_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return GPIOx->IDR & 0x00000001;
}

void My_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    // Check which pin was passed in and write to that one.
    if(PinState)
    {
        if(GPIO_Pin & GPIO_PIN_6)
            GPIOx->ODR |= 0x00000040;
        if(GPIO_Pin & GPIO_PIN_7)
            GPIOx->ODR |= 0x00000080;
        if(GPIO_Pin & GPIO_PIN_8)
            GPIOx->ODR |= 0x00000100;
        if(GPIO_Pin & GPIO_PIN_9)
            GPIOx->ODR |= 0x00000200;
    }
    else
    {
        if(GPIO_Pin & GPIO_PIN_6)
            GPIOx->ODR &= ~0x00000040;
        if(GPIO_Pin & GPIO_PIN_7)
            GPIOx->ODR &= ~0x00000080;
        if(GPIO_Pin & GPIO_PIN_8)
            GPIOx->ODR &= ~0x00000100;
        if(GPIO_Pin & GPIO_PIN_9)
            GPIOx->ODR &= ~0x00000200;
    }
}

void My_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR ^= GPIO_Pin;

    // Code for spinning.
    // if(GPIOx->ODR == 0x00000040)
    //     GPIOx->ODR = 0x00000200;
    // else if(GPIOx->ODR == 0x00000200)
    //     GPIOx->ODR = 0x00000080;
    // else if(GPIOx->ODR == 0x00000080)
    //     GPIOx->ODR = 0X00000100;
    // else
    //     GPIOx->ODR = 0x00000040;
}

void EnableInterruptForUserButton()
{
    EXTI->RTSR |= 0x1;
    EXTI->IMR |= 0x1;
}
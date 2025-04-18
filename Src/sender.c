#include "main.h"
#include <stm32f0xx_hal.h>

int sender_main(void)
{
    HAL_Init(); // Reset all peripherals

    gyroscope();


    return 1;
}
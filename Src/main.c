#include "main.h"
#include <stm32f0xx_hal.h>

int main(void)
{
    #if defined(RECEIVER)
        receiver_main();
    #else
        sender_main();
    #endif
}
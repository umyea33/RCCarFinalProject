#include <stm32f0xx_hal.h>
#include <nrf24l01p.h>

#define TRIG_PORT GPIOC
#define TRIG_PIN  GPIO_PIN_6

TIM_HandleTypeDef htim3;
volatile uint32_t ic_val1 = 0;
volatile uint32_t ic_val2 = 0;
volatile uint8_t is_first_capture = 0;
volatile uint32_t distance_cm = 0;

void send_trigger_pulse()
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    HAL_Delay(1); // >10us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (is_first_capture == 0)
        {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            is_first_capture = 1;
        }
        else
        {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            uint32_t diff = (ic_val2 >= ic_val1) ? (ic_val2 - ic_val1) : (0xFFFF - ic_val1 + ic_val2);
            distance_cm = diff * 0.034 / 2; // Speed of sound = 343 m/s
            is_first_capture = 0;
        }
    }
}

void ultrasonic_gpio_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    GPIO_InitTypeDef initC6 = 
    {
        GPIO_PIN_6,
        GPIO_MODE_AF_PP,
        GPIO_SPEED_FREQ_HIGH,
        GPIO_NOPULL
    };
    initC6.Alternate = GPIO_AF0_TIM3; // AF0 is correct for TIM3_CH1 on PC6

    GPIO_InitTypeDef initC7 = 
    {
        GPIO_PIN_7,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_LOW,
        GPIO_NOPULL
    };
    
    // TRIG = PC6 (AF0)
    HAL_GPIO_Init(GPIOC, &initC6);
    
    // Set to AF0

    // ECHO = PC7 (Output)
    HAL_GPIO_Init(GPIOC, &initC7);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1;         // 1 MHz tick = 1 µs
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim3);

    TIM_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

uint32_t ultrasonic_get_distance_cm(void)
{
    send_trigger_pulse();
    HAL_Delay(50); // Wait for echo capture (adjust as needed)
    return distance_cm;
}
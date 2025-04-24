#include <stm32f0xx_hal.h>
#include <assert.h>
#include <main.h>
#include <nrf24l01p.h>

uint8_t txData[NRF24L01P_PAYLOAD_LENGTH] = "worl";

int gyroscope (void) 
{
    // Set up the global x and y.
    int x = 0;
    int y = 0;
    int xleftovers = 0;
    int yleftovers = 0;

    while(1)
    {
        // Read the four registers of x and y from the gyroscope.
        uint8_t xh = read(0x29);
        uint8_t xl = read(0x28);
        uint8_t yh = read(0x2b);
        uint8_t yl = read(0x2a);

        txData[0] = 0;
        txData[1] = 0;
        txData[2] = 0;
        txData[3] = 0;

        // Update the global x and y based on the read values.
        (xh | 0x80) << 56;
        (xh | 0x7f) << 8;
        volatile int16_t xGyro = (xh << 8) | xl;
        volatile int16_t yGyro = (yh << 8) | yl;
        if(xGyro > -200 && xGyro < 200)
            xGyro = 0;
        if(yGyro > -200 && yGyro < 200)
            yGyro = 0;

        int xGyroAndLeftovers = xGyro + xleftovers;
        int yGyroAndLeftovers = yGyro + yleftovers;

        xleftovers = xGyroAndLeftovers % 5000;
        yleftovers = yGyroAndLeftovers % 5000;

        int xReduced = xGyroAndLeftovers / 5000;
        int yReduced = yGyroAndLeftovers / 5000;
        x += xReduced;
        y += yReduced;

        // Update the LEDs based on the new x and y.
        if(x >= 500)
        {
            My_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            My_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
            
        }
        else if(x < 500 && x > -500)
        {
            My_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            My_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
        }
        else
        {
            My_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
            My_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
        }

        if(y >= 500)
        {
            My_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
            My_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
        }
        else if(y < 500 && y > -500)
        {
            My_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
            My_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
        }
        else
        {
            My_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
            My_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
        }

        if(transmitData(txData) == 1)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
            HAL_Delay(1400);
        }
        else
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); 
        }

        // Wait 100 ms
        // HAL_Delay(10);
    }
}

void write(char data)
{
    // Wait until either the TXIS or NACKF bit is set
    while(!((I2C2->ISR & (1 << 1)) || (I2C2->ISR & (1 << 4))))
    {}

    // If the NACKF bit is set, there is a problem.  Turn on the orange LED.
    if(I2C2->ISR & (1 << 4))
    {
        My_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
        return;
    }

    I2C2->TXDR = data;
}

int8_t read(char reg)
{
    // Write //////////////////////////////////////////////////////////////////////////////
    // Set the transaction parameters in the CR2 register
    // Set the L3GD20 slave address 110101
    I2C2->CR2 = 0x69 << 1;
    I2C2->CR2 &= ~((0x7F << 16));
    I2C2->CR2 |= (1 << 16);
    I2C2->CR2 &= ~(1 << 10);
    I2C2->CR2 |= (1 << 13);

    // Wait until either the TXIS or NACKF bit is set
    while((!(I2C2->ISR & (1 << 1)) & !(I2C2->ISR & (1 << 4))))
    {}
    
    // If the NACKF bit is set, there is probably a problem.  Turn on the Red LED.
    if(I2C2->ISR & (1 << 4))
    {
        My_GPIO_WritePin(GPIOC, GPIO_PIN_6);
        My_GPIO_WritePin(GPIOC, GPIO_PIN_7);
        My_GPIO_WritePin(GPIOC, GPIO_PIN_8);
        My_GPIO_WritePin(GPIOC, GPIO_PIN_9);
        return;
    }

    // Write the address of the WHO_AM_I register into the I2C transmit register
    I2C2->TXDR = reg;

    // Wait until the Transfer Complete flag is set
    while(!(I2C2->ISR & (1 << 6)))
    {}

    // Read ///////////////////////////////////////////////////////////////////////////////
    // Reload the CR2 register with the same bits as before, but use read instead of write.
    I2C2->CR2 = 0x69 << 1;
    I2C2->CR2 |= (1 << 16);
    I2C2->CR2 &= ~((0x7F << 17));
    I2C2->CR2 |= (1 << 10);
    I2C2->CR2 |= (1 << 13);

    // Wait until either the RXNE or NACKF bit is set
    while(!((I2C2->ISR & (1 << 2)) || (I2C2->ISR & (1 << 4))))
    {}

    // If the NACKF bit is set, there is a problem.  Turn on all the LEDs
    if(I2C2->ISR & (1 << 4))
    {
        My_GPIO_WritePin(GPIOC, GPIO_PIN_6);
        My_GPIO_WritePin(GPIOC, GPIO_PIN_7);
        My_GPIO_WritePin(GPIOC, GPIO_PIN_8);
        My_GPIO_WritePin(GPIOC, GPIO_PIN_9);
        return;
    }

    int8_t result = I2C2->RXDR;

    // Wait until the Transfer Complete flag is set
    while(!(I2C2->ISR & (1 << 6)))
    {}

    return result;
}

void gyroInit(void)
{
// Set up a configuration struct to pass to the initialization function
GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
    GPIO_MODE_OUTPUT_PP,
    GPIO_SPEED_FREQ_LOW,
    GPIO_NOPULL};

    // Initialize GPIOC registers (The LEDs)
    My_GPIO_Init(GPIOC, &initStr);

    // Enable GPIOB in the RCC
    RCC->AHBENR |= 1 << 18;

    // My_GPIO_WritePin(GPIOC, GPIO_PIN_7);
    My_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
    My_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
    My_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
    My_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);

    HAL_Delay(1000);
    
    My_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
    My_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
    My_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
    My_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);

    // if((RCC->AHBENR & 1 << 19) && (RCC->AHBENR & 1 << 18))
    //     My_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    
    // Enable the I2C2 peripheral in the RCC
    RCC->APB1ENR |= (1 << 22);

    // Set PB11 to alternate function mode and open-drain output type.
    GPIOB->MODER |= (1 << 23);
    GPIOB->MODER &= ~(1 << 22);
    GPIOB->OTYPER |= (1 << 11);

    // Set I2C2-SDA as PB11's alternate function
    GPIOB->AFR[1] |= (1 << 12);
    GPIOB->AFR[1] &= ~((1 << 13) | (1 << 14) | (1 << 15));

    // Set PB13 to alternate fucntion mode and open-drain output type.
    GPIOB->MODER |= (1 << 27);
    GPIOB->MODER &= ~(1 << 26);
    GPIOB->OTYPER |= (1 << 13);

    // Set I2C2-SCL as PB13's alternate function
    GPIOB->AFR[1] |= (1 << 22) | (1 << 20);
    GPIOB->AFR[1] &= ~((1 << 23) | (1 << 21));

    // Set PB14 to output mode, push-pull output type, and initialize/set the pin high.
    GPIOB->MODER |= (1 << 28);
    GPIOB->MODER &= ~(1 << 29);
    GPIOB->OTYPER &= ~(1 << 14);
    GPIOB->ODR |= (1 << 14);

    // Check if MODER OTYPER and ODR are correct
    // MODER should be 0x18800000
    // if(GPIOB->MODER == 0x18800000)
    //     My_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    // if(GPIOB->OTYPER == 0x2800)
    //     My_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    // if(GPIOB->ODR == 0x4000)
    //     My_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    

    // Set PC0 to output mode, push-pull output type, and initialize/set the pin high.
    GPIOC->MODER |= (1 << 0);
    GPIOC->MODER &= ~(1 << 1);
    GPIOC->OTYPER &= ~(1 << 0);
    GPIOC->ODR |= (1 << 0);


    // Set the parameters in the TIMINGR register to use 100kHz standard-mode I2C.
    I2C2->TIMINGR |= (1 << 28);
    I2C2->TIMINGR &= ~(7 << 29);
    I2C2->TIMINGR |= (1 << 4) | (3 << 0);
    I2C2->TIMINGR &= ~(7 << 5) | (3 << 2);
    I2C2->TIMINGR |= (0xf << 8);
    I2C2->TIMINGR &= ~(0xf << 12);
    I2C2->TIMINGR |= (1 << 17);
    I2C2->TIMINGR &= ~((1 << 16) | (3 << 18));
    I2C2->TIMINGR |= (1 << 22);
    I2C2->TIMINGR &= ~((1 << 23) | (3 << 20));

    // Enable the I2C peripheral using the PE bit in the CR1 register
    I2C2->CR1 |= 1;

    // Set the transaction parameters in the CR2 register
    // Set the L3GD20 slave address 110101
    I2C2->CR2 = 0x69 << 1;
    I2C2->CR2 &= ~((0x7F << 16));
    I2C2->CR2 |= (1 << 16);
    I2C2->CR2 &= ~(1 << 10);
    I2C2->CR2 |= (1 << 13);

    // if(I2C2->CR2 == 0x12069)
    //     My_GPIO_TogglePin(GPIOC, GPIO_PIN_7);

    // Wait until either the TXIS or NACKF bit is set
    while((!(I2C2->ISR & (1 << 1)) & !(I2C2->ISR & (1 << 4))))
    {}

    // if(I2C2->ISR & (1 << 1))
    //     My_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    
    // If the NACKF bit is set, there is probably a problem.  Turn on the Red LED.
    if(I2C2->ISR & (1 << 4))
    {
        My_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        return;
    }

    // Write the address of the WHO_AM_I register into the I2C transmit register
    I2C2->TXDR = 0x0f;

    // Wait until the Transfer Complete flag is set
    while(!(I2C2->ISR & (1 << 6)))
    {}

    // Reload the CR2 register with the same bits as before, but use read instead of write.
    I2C2->CR2 = 0x69 << 1;
    I2C2->CR2 |= (1 << 16);
    I2C2->CR2 &= ~((0x7F << 17));
    I2C2->CR2 |= (1 << 10);
    I2C2->CR2 |= (1 << 13);

    // Wait until either the RXNE or NACKF bit is set
    while(!((I2C2->ISR & (1 << 2)) || (I2C2->ISR & (1 << 4))))
    {}

    // If the NACKF bit is set, there is a problem.  Turn on the orange LED.
    if(I2C2->ISR & (1 << 4))
    {
        My_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        return;
    }

    char whoAmI = I2C2->RXDR;

    // Wait until the Transfer Complete flag is set
    while(!(I2C2->ISR & (1 << 6)))
    {}
    
    // If the recieve register contains the right address, then turn on the blue and green LEDs.
    if(whoAmI == 0xD3)
        My_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_9);

    I2C2->CR2 |= (1 << 14);

    // Part 2 //////////////////////////////////////////////////////////////////////////////////

    // Enable the x and y axes on the gyroscope.
    I2C2->CR2 = 0;
    I2C2->CR2 = 0x69 << 1;
    I2C2->CR2 |= (1 << 17);
    I2C2->CR2 &= ~(1 << 10);
    I2C2->CR2 |= (1 << 13);

    write(0x20);
    write(0xb);

    // Wait until the Transfer Complete flag is set
    while(!(I2C2->ISR & (1 << 6)))
    {}
}
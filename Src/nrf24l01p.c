/*
 *  nrf24l01_plus.c
 *
 *  Created on: 4/24/2025.
 *      Author: Elden Harrison
 * 
 */


#include "nrf24l01p.h"
#include <stm32f0xx_hal.h>


void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

void setupSPI()
{
    GPIO_InitTypeDef initBRxIRQandCE = 
    {
        GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8,
        GPIO_MODE_OUTPUT_PP,
        GPIO_SPEED_FREQ_HIGH,
        GPIO_PULLUP
    };

    GPIO_InitTypeDef initBTransceiverAF = 
    {
        GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
        GPIO_MODE_AF_PP,
        GPIO_SPEED_FREQ_HIGH,
        GPIO_NOPULL
    };

    HAL_GPIO_Init(GPIOB, &initBRxIRQandCE);
    HAL_GPIO_Init(GPIOB, &initBTransceiverAF);

    // Set to alternate function af0
    GPIOB->AFR[0] &= ~(0xFFFFF000);
    GPIOB->AFR[1] &= ~(0xF);

    // Enable spi1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // SPI1 setup: master, BR=Fclk/16, CPOL=0, CPHA=1
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPHA;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 &= ~(1 << 7); // Most significant bit first
    SPI1->CR1 &= ~(1 << 10); // rxonly off full duplex mode
    SPI1->CR1 &= ~(1 << 11); //dff = 0 and 8-bit data
    SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1->CR2 &= ~SPI_CR2_FRF;
    SPI1->CR1 |= SPI_CR1_SPE;
    MX_SPI1_Init();
}

void writeReg(uint8_t reg, uint8_t data)
{
    uint8_t buff[2];
    buff[0] =  NRF24L01P_CMD_W_REGISTER | reg;
    buff[1] = data;

    // Pull cs pin low to select the slave device
    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, buff, 2, 1000);

    // Pull cs pin high
    cs_high();
}

void writeRegMulti(uint8_t reg, uint8_t* data, int size)
{
    uint8_t buff[2];
    buff[0] =  NRF24L01P_CMD_W_REGISTER | reg;

    // Pull cs pin low to select the slave device
    cs_low();

    HAL_SPI_Transmit(NRF24L01P_SPI, buff, 1, 100);
    HAL_SPI_Transmit(NRF24L01P_SPI, data, size, 1000);

    // Pull cs pin high
    cs_high();
}

uint8_t readReg(uint8_t reg)
{
    uint8_t data = 0;
    // Pull cs pin low to select the slave device
    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, &reg, 1, 100);
    HAL_SPI_Receive(NRF24L01P_SPI, &data, 1, 100);

    // Pull cs pin high
    cs_high();

    return data;
}

void readRegMulti(uint8_t reg, uint8_t* data, int size)
{
    // Pull cs pin low to select the slave device
    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, &reg, 1, 100);
    HAL_SPI_Receive(NRF24L01P_SPI, data, size, 1000);

    // Pull cs pin high
    cs_high();
}

void nrfSendCmd(uint8_t cmd)
{
    // Pull cs pin low to select the slave device
    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, &cmd, 1, 1000);

    // Pull cs pin high
    cs_high();
}

void nrf24Init()
{
    // Disable the chip before configuring.
    ce_low();

    nrf24Reset(0);

    writeReg(NRF24L01P_REG_CONFIG, 0); // configured later

    writeReg(NRF24L01P_REG_EN_AA, 0); // No auto acknowledge

    writeReg(NRF24L01P_REG_EN_RXADDR, (1 << 1)); // Not enabling any data pipe right now

    writeReg(NRF24L01P_REG_SETUP_AW, 3); // 5 bytes for tx/rx address

    writeReg(NRF24L01P_REG_SETUP_RETR, 0); // No retransmission

    writeReg(NRF24L01P_REG_RF_CH, 0); // will set up during tx or rx
    
    uint8_t new_rf_setup = readReg(NRF24L01P_REG_RF_SETUP) & 0xD7;
    new_rf_setup |= (1 << 5) | (1 << 2) | (1 << 1); 

    writeReg(NRF24L01P_REG_RF_SETUP, new_rf_setup); // Power = 0db, data rate = 250kbps

    // Re-enable the device.
    ce_high();
}

// Setup TX mode
void nrf24TxMode(uint8_t* address, uint8_t channel)
{
    // Disable the chip before configuring.
    ce_low();

    writeReg(NRF24L01P_REG_RF_CH, channel); // Select the channel
    
    writeRegMulti(NRF24L01P_REG_TX_ADDR, address, 5); // Write the tx

    // Power up the device.
    uint8_t config = readReg(NRF24L01P_REG_CONFIG);
    config = config | (1 << 1);
    writeReg(NRF24L01P_REG_CONFIG, config);

    // Re-enable the device.
    ce_high();
}

// Transmit Data
uint8_t transmitData(uint8_t* data)
{
    uint8_t sendCmd = 0;
    
    cs_low();

    // Payload cmd
    sendCmd = NRF24L01P_CMD_W_TX_PAYLOAD;
    HAL_SPI_Transmit(NRF24L01P_SPI, &sendCmd, 1, 100);

    // Send payload
    HAL_SPI_Transmit(NRF24L01P_SPI, data, NRF24L01P_PAYLOAD_LENGTH, 1000);

    // Unselect the device
    cs_high();

    HAL_Delay(1);

    // Check fifo status to know if tx fifo is empty.
    uint8_t status = readReg(NRF24L01P_REG_FIFO_STATUS);
    if((status & (1 << 4)) && !(status & (1 << 3)))
    {
        sendCmd = NRF24L01P_CMD_FLUSH_TX;
        nrfSendCmd(sendCmd);
        nrf24Reset(NRF24L01P_REG_STATUS);
        // writeReg(NRF24L01P_REG_STATUS, readReg(NRF24L01P_REG_STATUS) | (1 << 5));
        return 1;
    }

    return 0;
}

void nrfRxMode(uint8_t* address, uint8_t channel)
{
    // Disable chip before configuring.
    ce_low();

    // Reset
    nrf24Reset(NRF24L01P_REG_STATUS);

    writeReg(NRF24L01P_REG_RF_CH, channel); // Select Channel

    // Select Data Pipe 1
    uint8_t pipe = readReg(NRF24L01P_REG_EN_RXADDR) | (1 << 1);
    writeReg(NRF24L01P_REG_EN_RXADDR, pipe);

    writeRegMulti(NRF24L01P_REG_RX_ADDR_P1, address, 5); // Write the rx address.

    writeReg(NRF24L01P_REG_RX_PW_P1, NRF24L01P_PAYLOAD_LENGTH); // # of bit payload for pipe 1.

    uint8_t newConfig = readReg(NRF24L01P_REG_CONFIG) | (1 << 1) | (1 << 0);
    writeReg(NRF24L01P_REG_CONFIG, newConfig);

    ce_high();
}

uint8_t isDataAvailable(int pipenum)
{
    uint8_t status = readReg(NRF24L01P_REG_STATUS);

    if((status & (1 << 6)) && (status & (pipenum << 1)))
    {
        writeReg(NRF24L01P_REG_STATUS, (1 << 6));
        return 1;
    }

    return 0;
}

void receiveData(uint8_t* data)
{
    uint8_t sendCmd = 0;
    cs_low();

    // Payload cmd
    sendCmd = NRF24L01P_CMD_R_RX_PAYLOAD;
    HAL_SPI_Transmit(NRF24L01P_SPI, &sendCmd, 1, 100);

    // Send payload
    HAL_SPI_Receive(NRF24L01P_SPI, data, NRF24L01P_PAYLOAD_LENGTH, 1000);

    // Unselect the device
    cs_high();

    HAL_Delay(1);

    sendCmd = NRF24L01P_CMD_FLUSH_RX;
    nrfSendCmd(sendCmd);
}

void nrf24Reset(uint8_t reg)
{
	if (reg == NRF24L01P_REG_STATUS)
	{
		writeReg(NRF24L01P_REG_STATUS, 0x00);
	}
	else if (reg == NRF24L01P_REG_FIFO_STATUS)
	{
		writeReg(NRF24L01P_REG_FIFO_STATUS, 0x11);
	}
	else
    {
        writeReg(NRF24L01P_REG_CONFIG, 0x08);
        writeReg(NRF24L01P_REG_EN_AA, 0x3F);
        writeReg(NRF24L01P_REG_EN_RXADDR, 0x03);
        writeReg(NRF24L01P_REG_SETUP_AW, 0x03);
        writeReg(NRF24L01P_REG_SETUP_RETR, 0x03);
        writeReg(NRF24L01P_REG_RF_CH, 0x02);
        writeReg(NRF24L01P_REG_RF_SETUP, 0x0E);
        writeReg(NRF24L01P_REG_STATUS, 0x00);
        writeReg(NRF24L01P_REG_OBSERVE_TX, 0x00);
        writeReg(NRF24L01P_REG_RPD, 0x00);
        uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        writeRegMulti(NRF24L01P_REG_RX_ADDR_P0, rx_addr_p0_def, 5);
        uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
        writeRegMulti(NRF24L01P_REG_RX_ADDR_P1, rx_addr_p1_def, 5);
        writeReg(NRF24L01P_REG_RX_ADDR_P2, 0xC3);
        writeReg(NRF24L01P_REG_RX_ADDR_P3, 0xC4);
        writeReg(NRF24L01P_REG_RX_ADDR_P4, 0xC5);
        writeReg(NRF24L01P_REG_RX_ADDR_P5, 0xC6);
        uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        writeRegMulti(NRF24L01P_REG_TX_ADDR, tx_addr_def, 5);
        writeReg(NRF24L01P_REG_RX_PW_P0, 0);
        writeReg(NRF24L01P_REG_RX_PW_P1, 0);
        writeReg(NRF24L01P_REG_RX_PW_P2, 0);
        writeReg(NRF24L01P_REG_RX_PW_P3, 0);
        writeReg(NRF24L01P_REG_RX_PW_P4, 0);
        writeReg(NRF24L01P_REG_RX_PW_P5, 0);
        writeReg(NRF24L01P_REG_FIFO_STATUS, 0x11);
        writeReg(NRF24L01P_REG_DYNPD, 0);
        writeReg(NRF24L01P_REG_FEATURE, 0);
	}
}

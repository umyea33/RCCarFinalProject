/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
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
        nrf24Reset(NRF24L01P_REG_FIFO_STATUS);
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



























//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t read_register(uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, &read_val, 1, 2000);
    cs_high();

    return read_val;
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, &write_val, 1, 2000);
    cs_high();

    return write_val;
}


/* nRF24L01+ Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_prx_mode();
    nrf24l01p_power_up();

    nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);
    
    ce_high();
}

void nrf24l01p_tx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_ptx_mode();
    nrf24l01p_power_up();

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(1);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);

    ce_high();
}

void nrf24l01p_rx_receive(uint8_t* rx_payload)
{
    nrf24l01p_read_rx_fifo(rx_payload);
    nrf24l01p_clear_rx_dr();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void nrf24l01p_tx_transmit(uint8_t* tx_payload)
{
    nrf24l01p_write_tx_fifo(tx_payload);
}

void nrf24l01p_tx_irq()
{
    uint8_t tx_ds = nrf24l01p_get_status();
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        nrf24l01p_clear_tx_ds();
    }

    else
    {
        // MAX_RT
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        nrf24l01p_clear_max_rt();
    }
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Reset registers
    write_register(NRF24L01P_REG_CONFIG, 0x08);
    write_register(NRF24L01P_REG_EN_AA, 0x3F);
    write_register(NRF24L01P_REG_EN_RXADDR, 0x03);
    write_register(NRF24L01P_REG_SETUP_AW, 0x03);
    write_register(NRF24L01P_REG_SETUP_RETR, 0x03);
    write_register(NRF24L01P_REG_RF_CH, 0x02);
    write_register(NRF24L01P_REG_RF_SETUP, 0x07);
    write_register(NRF24L01P_REG_STATUS, 0x7E);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P1, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P5, 0x00);
    write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
    write_register(NRF24L01P_REG_DYNPD, 0x00);
    write_register(NRF24L01P_REG_FEATURE, 0x00);

    // Reset FIFO
    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_prx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();

    return status;
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high(); 

    return status;
}

void nrf24l01p_flush_rx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_flush_tx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

uint8_t nrf24l01p_get_status()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high(); 

    return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_widths(widths bytes)
{
    write_register(NRF24L01P_REG_RX_PW_P0, bytes);
}

void nrf24l01p_clear_rx_dr()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x40;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x20;

    write_register(NRF24L01P_REG_STATUS, new_status);     
}

void nrf24l01p_clear_max_rt()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x10;

    write_register(NRF24L01P_REG_STATUS, new_status); 
}

void nrf24l01p_power_up()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(length bytes)
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(widths bytes)
{
    write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}
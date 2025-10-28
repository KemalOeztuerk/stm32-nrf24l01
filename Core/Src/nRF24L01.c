/*
 * nRF24L01.c
 *
 *  Created on: Oct 26, 2025
 *      Author: kemal
 */

#include "nRF24L01.h"
#include <stdio.h>

void set_csn_high(nrf_t *nrf){
	HAL_GPIO_WritePin(nrf->csn_port, nrf->csn_pin, 1);
}

void set_csn_low(nrf_t *nrf){
	HAL_GPIO_WritePin(nrf->csn_port, nrf->csn_pin, 0);
}

void set_ce_high(nrf_t *nrf){
	HAL_GPIO_WritePin(nrf->ce_port, nrf->ce_pin, 1);
}

void set_ce_low(nrf_t *nrf){
	HAL_GPIO_WritePin(nrf->ce_port, nrf->ce_pin, 0);
}

void nrf_write_reg(nrf_t *nrf, uint8_t reg, uint8_t data){

	uint8_t cmd = NRF_CMD_W_REGISTER | reg;
	set_csn_low(nrf);
	HAL_SPI_Transmit(nrf->spi, &cmd, 1, 100);
	HAL_SPI_Transmit(nrf->spi, &data, 1, 100);
	set_csn_high(nrf);
}

uint8_t nrf_read_reg(nrf_t *nrf, uint8_t reg){
	uint8_t cmd = NRF_CMD_R_REGISTER | reg;
	uint8_t data;
	set_csn_low(nrf);
	HAL_SPI_Transmit(nrf->spi, &cmd, 1, 100);
	HAL_SPI_Receive(nrf->spi, &data, 1, 100);
	set_csn_high(nrf);
	return data;
}

void nrf_write_reg_multi(nrf_t *nrf, uint8_t reg, uint16_t size, uint8_t *data){
	uint8_t cmd = NRF_CMD_W_REGISTER | reg;
	set_csn_low(nrf);
	HAL_SPI_Transmit(nrf->spi, &cmd, 1, 100);
	HAL_SPI_Transmit(nrf->spi, data, size, 100);
	set_csn_high(nrf);
}

void nrf_read_reg_multi(nrf_t *nrf, uint8_t reg, uint16_t size, uint8_t *data){
	uint8_t cmd = NRF_CMD_R_REGISTER | reg;
	set_csn_low(nrf);
	HAL_SPI_Transmit(nrf->spi, &cmd, 1, 100);
	HAL_SPI_Receive(nrf->spi, data, size, 100);
	set_csn_high(nrf);
}

void nrf_send_cmd(nrf_t *nrf, uint8_t cmd) {
    set_csn_low(nrf);
    HAL_SPI_Transmit(nrf->spi, &cmd, 1, 100);
    set_csn_high(nrf);
}

/*int nrf_init_default(nrf_t *nrf){
	set_ce_low(nrf);
	nrf_write_reg(nrf, NRF_REG_CONFIG, 0x00);
	nrf_write_reg(nrf, NRF_REG_EN_AA, 0x00);
	nrf_write_reg(nrf, NRF_REG_EN_RXADDR, 1);
	nrf_write_reg(nrf, NRF_REG_SETUP_AW, 0b10);
	nrf_write_reg(nrf, NRF_REG_SETUP_RETR, 0x00);
	nrf_write_reg(nrf, NRF_REG_RF_CH, 1);
	nrf_write_reg(nrf, NRF_REG_RF_SETUP, 0x00);
	set_csn_high(nrf);
	return 0;
}
*/
int nrf_init_tx(nrf_t *nrf, uint8_t *tx_addr){
	set_ce_low(nrf);
	set_csn_high(nrf);
    HAL_Delay(100);

	nrf_write_reg(nrf, NRF_REG_CONFIG, 0x00);
	nrf_write_reg(nrf, NRF_REG_EN_AA, 0x00);
	nrf_write_reg(nrf, NRF_REG_EN_RXADDR, 0x00); // ???
	nrf_write_reg(nrf, NRF_REG_SETUP_AW, 0b11);
	nrf_write_reg(nrf, NRF_REG_SETUP_RETR, 0x00);
	nrf_write_reg(nrf, NRF_REG_RF_CH, 1); // setting channel 1
	nrf_write_reg(nrf, NRF_REG_RF_SETUP, 0x07);
	nrf_write_reg_multi(nrf, NRF_REG_TX_ADDR, 5, tx_addr);
	nrf_write_reg(nrf, NRF_REG_RX_PW_P0, 32);
	nrf_send_cmd(nrf, NRF_CMD_FLUSH_TX);
	nrf_send_cmd(nrf, NRF_CMD_FLUSH_RX);
	nrf_write_reg(nrf, NRF_REG_CONFIG, 0b00000010); // POWER UP
	HAL_Delay(10);
	return 0;
}


int nrf_transmit(nrf_t *nrf, uint8_t *data, uint16_t size){
	uint8_t status;
	uint32_t start_time = HAL_GetTick();
	set_csn_low(nrf);
	uint8_t cmd = NRF_CMD_W_TX_PAYLOAD;
	HAL_SPI_Transmit(nrf->spi, &cmd, 1, 100);
	HAL_SPI_Transmit(nrf->spi, data, size, 100);
	set_csn_high(nrf);

	set_ce_high(nrf);
	HAL_Delay(1); // 1ms pulse (min is 10us, 1ms is very safe)
	set_ce_low(nrf);

	/*uint8_t status = nrf_read_reg(nrf, NRF_REG_FIFO_STATUS);
	if((status & (1<<4))==1){
		cmd = NRF_CMD_FLUSH_TX;
		nrf_send_cmd(nrf, cmd);
		printf("message could not transmitted\n");
		return 1;
	}
	*/

	// 3. *** CRITICAL FIX: CHECK NRF_REG_STATUS, NOT FIFO_STATUS ***
	    while(1) {
	        // Read the main STATUS register (0x07)
	        status = nrf_read_reg(nrf, NRF_REG_STATUS);

	        // --- SUCCESS: Data Sent (TX_DS) ---
	        if (status & (1 << 5)) {
	            nrf_write_reg(nrf, NRF_REG_STATUS, (1 << 5)); // Clear flag
	            return 0; // Success
	        }

	        // --- FAILURE: Max Retries (MAX_RT) ---
	        if (status & (1 << 4)) {
	            printf("message could not be transmitted (MAX_RT)\n");
	            nrf_write_reg(nrf, NRF_REG_STATUS, (1 << 4)); // Clear flag
	            nrf_send_cmd(nrf, NRF_CMD_FLUSH_TX); // Flush the failed packet
	            return 1; // Failure
	        }

	        // --- TIMEOUT ---
	        if (HAL_GetTick() - start_time > 100) { // 100ms timeout
	            printf("message could not be transmitted (timeout)\n");
	            return 1; // Failure
	        }
	    }
	return 0;
}

int nrf_init_rx(nrf_t *nrf, uint8_t *rx_addr){
	set_ce_low(nrf);
	set_csn_high(nrf);
    HAL_Delay(100);

	nrf_write_reg(nrf, NRF_REG_CONFIG, 0x00);
	nrf_write_reg(nrf, NRF_REG_EN_AA, 0x00);
	nrf_write_reg(nrf, NRF_REG_EN_RXADDR, 0x01);
	nrf_write_reg(nrf, NRF_REG_SETUP_AW, 0b11);
	nrf_write_reg(nrf, NRF_REG_SETUP_RETR, 0x00);
	nrf_write_reg(nrf, NRF_REG_RF_CH, 1);
	nrf_write_reg(nrf, NRF_REG_RF_SETUP, 0x07);
	nrf_write_reg_multi(nrf, NRF_REG_RX_ADDR_P0, 5, rx_addr);
	nrf_write_reg(nrf,NRF_REG_RX_PW_P0,32); // set payload size 32 byte
	nrf_send_cmd(nrf, NRF_CMD_FLUSH_TX);
	nrf_send_cmd(nrf, NRF_CMD_FLUSH_RX);
	nrf_write_reg(nrf, NRF_REG_CONFIG, 0b00000011); // POWER UP, SET RX MODE
	HAL_Delay(10);
	set_ce_high(nrf);

	return 0;
}

int nrf_receive(nrf_t *nrf, uint8_t *data) {
    uint8_t status;

    // 1. Check the STATUS register
    status = nrf_read_reg(nrf, NRF_REG_STATUS);

    // 2. Check if the RX_DR (Data Ready) flag is set (bit 6)
    if (status & (1 << 6)) {

        // 3. Read the payload from the RX FIFO
        uint8_t cmd = NRF_CMD_R_RX_PAYLOAD;
        set_csn_low(nrf);
        HAL_SPI_Transmit(nrf->spi, &cmd, 1, 100);
        HAL_SPI_Receive(nrf->spi, data, 32, 100);
        set_csn_high(nrf);

        // 4. Clear the RX_DR flag by writing '1' to it
        nrf_write_reg(nrf, NRF_REG_STATUS, (1 << 6));

        return 1; // New data received
    }

    return 0; // No new data
}




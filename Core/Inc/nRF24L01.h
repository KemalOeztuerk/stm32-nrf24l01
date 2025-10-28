/*
 * nRF24L01.h
 *
 *  Created on: Oct 26, 2025
 *      Author: kemal
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_
#include "stm32f4xx_hal.h"

#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_R_RX_PAYLOAD  0x61
#define NRF_CMD_W_TX_PAYLOAD  0xA0
#define NRF_CMD_FLUSH_TX      0xE1
#define NRF_CMD_FLUSH_RX      0xE2
#define NRF_CMD_REUSE_TX_PL   0xE3
#define NRF_CMD_NOP           0xFF


#define NRF_REG_CONFIG        0x00
#define NRF_REG_EN_AA         0x01
#define NRF_REG_EN_RXADDR     0x02
#define NRF_REG_SETUP_AW      0x03
#define NRF_REG_SETUP_RETR    0x04
#define NRF_REG_RF_CH         0x05
#define NRF_REG_RF_SETUP      0x06
#define NRF_REG_STATUS        0x07
#define NRF_REG_OBSERVE_TX    0x08
#define NRF_REG_RPD           0x09
#define NRF_REG_RX_ADDR_P0    0x0A
#define NRF_REG_RX_ADDR_P1    0x0B
#define NRF_REG_RX_ADDR_P2    0x0C
#define NRF_REG_RX_ADDR_P3    0x0D
#define NRF_REG_RX_ADDR_P4    0x0E
#define NRF_REG_RX_ADDR_P5    0x0F
#define NRF_REG_TX_ADDR       0x10
#define NRF_REG_RX_PW_P0      0x11
#define NRF_REG_RX_PW_P1      0x12
#define NRF_REG_RX_PW_P2      0x13
#define NRF_REG_RX_PW_P3      0x14
#define NRF_REG_RX_PW_P4      0x15
#define NRF_REG_RX_PW_P5      0x16
#define NRF_REG_FIFO_STATUS   0x17
#define NRF_REG_DYNPD         0x1C
#define NRF_REG_FEATURE       0x1D


typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef * csn_port;
    uint16_t      csn_pin;
    GPIO_TypeDef *ce_port;
    uint16_t      ce_pin;
    //uint8_t rf_channel;
    //uint8_t rx_addr; // enabled RX addresses. 0-5th channels can be enabled
    //char * msg;
} nrf_t;



void set_csn_high(nrf_t *nrf);
void set_csn_low(nrf_t *nrf);
void set_ce_high(nrf_t *nrf);
void set_ce_low(nrf_t *nrf);
void nrf_write_reg(nrf_t *nrf, uint8_t reg, uint8_t data);
uint8_t nrf_read_reg(nrf_t *nrf, uint8_t reg);
void nrf_write_reg_multi(nrf_t *nrf, uint8_t reg, uint16_t size, uint8_t *data);
void nrf_read_reg_multi(nrf_t *nrf, uint8_t reg, uint16_t size, uint8_t *data);
//int nrf_init_default(nrf_t *nrf);
int nrf_init_tx(nrf_t *nrf, uint8_t *tx_addr);
int nrf_transmit(nrf_t *nrf, uint8_t *data, uint16_t size);
int nrf_init_rx(nrf_t *nrf, uint8_t *rx_addr);
int nrf_receive(nrf_t *nrf, uint8_t *data);
void nrf_send_cmd(nrf_t *nrf, uint8_t cmd);





#endif /* INC_NRF24L01_H_ */

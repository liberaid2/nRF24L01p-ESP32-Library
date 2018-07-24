/*  
Copyright 2018 Shitov Egor

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
*/

#ifndef DRIVERS_NRF24L01P_HAL_H
#define DRIVERS_NRF24L01P_HAL_H

#define NRF_CMD_R_REGISTER		0x0
#define NRF_CMD_W_REGISTER 		0x20
#define NRF_CMD_R_RX_PAYLOAD	0x61
#define NRF_CMD_W_TX_PAYLOAD	0xA0
#define NRF_CMD_FLUSH_TX 		0xE1
#define NRF_CMD_FLUSH_RX 		0xE2
#define NRF_CMD_NOP				0xFF

#define NRF_REG_CONFIG 			0x00
#define NRF_REG_ENAA 			0x01
#define NRF_REG_ENRXADDR		0x02
#define NRF_REG_SETUP_ADDR_W	0x03
#define NRF_REG_SETUP_RETR		0x04
#define NRF_REG_SETUP_CHANNEL 	0x05
#define NRF_REG_SETUP			0x06
#define NRF_REG_STATUS			0x07
#define NRF_REG_OBSERVE_TX		0x08
#define NRF_REG_RX_ADDR_P0		0x0A
#define NRF_REG_RX_ADDR_P1		0x0B
#define NRF_REG_RX_ADDR_P2		0x0C
#define NRF_REG_RX_ADDR_P3		0x0D
#define NRF_REG_RX_ADDR_P4		0x0E
#define NRF_REG_RX_ADDR_P5		0x0F
#define NRF_REG_TX_ADDR			0x10

/* Payload widths for pipes */
#define NRF_REG_RX_PW_P0		0x11
#define NRF_REG_RX_PW_P1		0x12
#define NRF_REG_RX_PW_P2		0x13	
#define NRF_REG_RX_PW_P3		0x14	
#define NRF_REG_RX_PW_P4		0x15	
#define NRF_REG_RX_PW_P5		0x16
/* ------------------------ */

#define NRF_REG_FIFO_STATUS		0x17
#define NRF_REG_DYN_PW			0x1C
#define NRF_REG_FEATURE			0x1D

/* Due to endianess here we got reversed low and high bits */
#define NRF_DATA_RATE_250k		0b01
#define NRF_DATA_RATE_1m		0b00
#define NRF_DATA_RATE_2m		0b10

#define NRF_POWER_m18dBm		0b00
#define NRF_POWER_m12dBm		0b01
#define NRF_POWER_m6dBm			0b10
#define NRF_POWER_0dBm			0b00

#define NRF_REG_RPD				0x09

#include <inttypes.h>

/* Configuration register */
typedef struct {
	uint8_t prim_rx : 1;		/* 0 - tx mode, 1 - rx mode */
	uint8_t pwr_up : 1;			/* 0 - sleep mode, 1 - power up mode */
	uint8_t crco : 1;			/* crc length 0 - 1 byte, 1 - 2 bytes */
	uint8_t en_crc : 1;			/* 0 - crc enabled, 1 - crc disabled */
	uint8_t mask_max_rt : 1;	/* 0 - rt irq disabled, 1 - rt irq enabled */
	uint8_t mask_tx_ds : 1;		/* 0 - tx ds irq disabled, 1 - tx ds irq enabled */
	uint8_t mask_rx_dr : 1;		/* 0 - rx dr irq disabled, 1 - rx dr irq enabled */
	uint8_t reserved : 1;		/* reserved, only 0 allowed */
} nrf_reg_config_t;

/* Enabled auto acknowledgement
 * Should be disabled for nRF24L01 compatibility 
 * Enables aa for specific pipe */
typedef struct {
	uint8_t enaa_pipe0 : 1;
	uint8_t enaa_pipe1 : 1;
	uint8_t enaa_pipe2 : 1;
	uint8_t enaa_pipe3 : 1;
	uint8_t enaa_pipe4 : 1;
	uint8_t enaa_pipe5 : 1;
	uint8_t reserved : 2;		/* reserved, only 00 allowed */
} nrf_reg_en_aa_t;

/* Enable pipe for receiving */
typedef struct {
	uint8_t enrxaddr_pipe0 : 1;
	uint8_t enrxaddr_pipe1 : 1;
	uint8_t enrxaddr_pipe2 : 1;
	uint8_t enrxaddr_pipe3 : 1;
	uint8_t enrxaddr_pipe4 : 1;
	uint8_t enrxaddr_pipe5 : 1;
	uint8_t reserved : 2;		/* reserved, only 00 allowed */
} nrf_reg_en_rxaddr_t;

/* Configures address width */
typedef struct {
	uint8_t width : 2;			/* 00 - illegal, 01 - 3 bytes, 10 - 4 bytes, 11 - 5 bytes */
	uint8_t reserved : 6;		/* reserved, only 000000 allowed */
} nrf_reg_addr_width_t;

/* Configures retries count & delay */
typedef struct {
	uint8_t count : 4;
	uint8_t delay : 4;
} nrf_reg_retries_t;

typedef struct {
	uint8_t channel : 7;
	uint8_t reserved : 1;
} nrf_reg_channel_t;

typedef struct {
	uint8_t obsolete : 1;
	uint8_t power : 2;
	uint8_t dr_high : 1;
	uint8_t pll_lock : 1;
	uint8_t dr_low : 1;
	uint8_t reserved : 1;
	uint8_t cont_wave : 1;
} nrf_reg_setup_t;

/* Status register */
typedef struct {
	uint8_t tx_full : 1;		/* Tx full flag */
	uint8_t rx_p_n : 3;			/* Data pipe number for payload available for reading */
	uint8_t max_rt : 1;			/* Max retries flag */
	uint8_t tx_ds : 1;			/* Data sent flag */
	uint8_t rx_dr : 1;			/* Data received flag */
	uint8_t reserved : 1;
} nrf_reg_status_t;

typedef struct {
	uint8_t rt_count : 4;		/* Retries count */
	uint8_t plos_count : 4;		/* Lost packets count */
} nrf_reg_observe_tx_t;

/* Fifo status */
typedef struct {
	uint8_t rx_empty : 1;
	uint8_t rx_full : 1;
	uint8_t reserved : 1;
	uint8_t tx_full : 1;
	uint8_t tx_empty : 1;
	uint8_t tx_reuse : 1;
	uint8_t reserved2 : 1;
} nrf_reg_fifo_status_t;

#endif /* DRIVERS_NRF24L01P_HAL_H */
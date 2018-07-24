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

#ifndef DRIVERS_NRF24L01_LIB_H
#define DRIVERS_NRF24L01_LIB_H

#include "nrf24l01p_hal.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <string.h>

#ifndef DEBUG_MODE
// #define DEBUG_MODE
#endif /* DEBUG_MODE */

#ifdef DEBUG_MODE
#include "uart.h"
#endif

#define NRF_MAX_PAYLOAD		32 // in bytes
#define NRF_ENABLE_CRC		1
#define NRF_CRC_WIDTH		1

/* nRF24L01 modes
 * rx - receive mode
 * tx - send mode */
typedef enum{
	nrf_mode_none,
	nrf_rx_mode,
	nrf_tx_mode
} nrf_mode_t;

/* Transfer one byte via spi and get result
 * This function is expected to be used in full duplex mode
 * Take care of setting CS pin down before calling this function
 * 
 * This function is needed because native esp-idf spi master interface cannot transfer properly 
 * more than 1 byte. So we are sending data byte by byte literally */
static inline uint8_t spi_transfer_byte(uint8_t byte, spi_device_handle_t device){
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));

	t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
	t.length = 8;

	t.tx_data[0] = byte;

	spi_device_transmit(device, &t);
	return t.rx_data[0];
}

/* This library currently supports only fixed payload size (MAX_PAYLOAD)
 * Also by default all pipes are ready to receive data
 * And payload width is set to MAX_PAYLOAD in all pipes */
class CNRFLib{
public:
	CNRFLib(gpio_num_t cs, gpio_num_t ce) : m_CS(cs), m_CE(ce) {}
	~CNRFLib();

	/* Attach our device to spi bus
	 * It's expected that spi bus is already configured
	 * Returns ESP_OK on success, another esp error code otherwise */
	esp_err_t AttachToSpiBus(spi_host_device_t spiBus);

	/* Primary preparations for nRF24L01
	 * mode - set chip to receive or send mode
	 * 
	 * Mode can be changed further */
	void Begin(nrf_mode_t mode);

	/* Send chunk of data
	 * Length must be less or equal to 32 (in bytes) 
	 * 
	 * Note: If you want to use custom TXaddr 
	 * Setup TX Addr before using this function.
	 * Also this function expects mode to be nrf_tx_mode (actually sleeping mode)
	 * 
	 * Return: number of retries on success, -1 on fail */
	int8_t Send(uint8_t *pBuff, uint8_t length);

	/* Read chunk of data from rx buffer
	 * Length must be less or equal to 32 (in bytes)
	 * Also there must be data in rx buffer */
	void Read(uint8_t *pBuff, uint8_t length);

	/* Power down (minimal current consuming mode) */
	void SetSleepTxMode();
	/* Power up, receiving mode */
	void SetRxMode();

	/* Flush buffers */
	void FlushRX();
	void FlushTX();

	nrf_reg_status_t GetStatus();
	nrf_reg_fifo_status_t GetFifoStatus();

	/* Check available data in rx buffer for specific pipe 
	 * Return: true if there are any data available, false otherwise */
	bool IsRxDataAvailable(uint8_t pipeNo);

	/* Check available data in rx buffer for any pipes
	 * Return: true if there are any data available, false otherwise */
	bool IsRxDataAvailable();

	/* Configure auto retransmission option
	 * count - how many times chip will try to send data
	 * delay - 0 - disabled, from 1 to 15 - gradation between 250us and 4ms
	 * (To better understand this look at datasheet) */
	void SetAutoRetransmission(uint8_t count, uint8_t delay);

	/* Set channel */
	void SetChannel(uint8_t channel);

	/* Set dataRate (250kbps, 1mbps, 2mbps)
	 * Choose from NRF_DATA_RATE_* */
	void SetDataRate(uint8_t dataRate);

	/* Power in dBm, Choose from NRF_POWER_* */
	void SetPower(uint8_t power);

	/* Since dataRate and power are located in the same register
	 * We can make one request instead of two */
	void SetDataRateAndPower(uint8_t dataRate, uint8_t power);

	/* Set addr for specific pipe
	 * This library currently doesn't support 3-byte and 4-bytes addresses 
	 * So, for pipes 0 and 1 this function expects 5 byte addr 
	 * For pipes from 2 to 5 this function expects 1 byte addr 
	 * (To better understand how addrs for pipes from 2 to 5 works
	 *  look at datasheet) */
	void SetPipeAddr(uint8_t pipeNo, uint8_t *pAddr, uint8_t length);

	/* Get pipe addr. Look at the description above to understand pAddr length */
	void GetPipeAddr(uint8_t pipeNo, uint8_t *pAddr);

	/* Set Tx Addr (addr length must be 5 (in bytes) ) */
	void SetTxAddr(uint8_t *pAddr, uint8_t length);

	/* Get Tx Addr (addr length must be 5 (in bytes) ) */
	void GetTxAddr(uint8_t *pAddr);

	/* Scans all 128 channels
	 * Sets bit 0 when channel is free, bit 1 when channel is busy
	 * Busy channel means there are noises more than -64dBm
	 * It's better to use free channels */
	void ScanChannels(uint64_t &firstHalf, uint64_t &secondHalf);

#ifdef DEBUG_MODE
	void PrintRegister(nrf_reg_config_t config);
	void PrintRegister(nrf_reg_status_t status);
	void PrintRegister(nrf_reg_fifo_status_t status);
#endif
private:
	gpio_num_t m_CS;
	gpio_num_t m_CE;

	spi_device_handle_t m_SpiDevice;

	bool isAttached;
	nrf_mode_t m_Mode;

	uint8_t ReadReg(uint8_t reg);
	uint8_t WriteReg(uint8_t reg, uint8_t value);
	uint8_t ReadBytes(uint8_t cmd, uint8_t *pBuff, uint8_t length);
	uint8_t WriteBytes(uint8_t cmd, uint8_t *pBuff, uint8_t length);

	void WriteTxPayload(uint8_t *pBuff, uint8_t length);
	void ReadRxPayload(uint8_t *pBuff, uint8_t length);

	void CELow();
	void CEHigh();
	void CSLow();
	void CSHigh();
};

#endif /* DRIVERS_NRF24L01_LIB_H */
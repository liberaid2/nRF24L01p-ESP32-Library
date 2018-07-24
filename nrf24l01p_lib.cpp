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

#include "nrf24l01p_lib.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define DELAY_MS(x) vTaskDelay(pdMS_TO_TICKS((x)))
#define _TOUINT(x) (*((uint8_t *)(&(x))))

CNRFLib::~CNRFLib(){
	if(isAttached)
		spi_bus_remove_device(m_SpiDevice);
}

esp_err_t CNRFLib::AttachToSpiBus(spi_host_device_t spiBus){
	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));

	devcfg.clock_speed_hz = 1 * 1000 * 1000;
	devcfg.mode = 0;
	devcfg.spics_io_num = -1;
	devcfg.queue_size = 8;
	devcfg.duty_cycle_pos = 128;

	esp_err_t err = spi_bus_add_device(spiBus, &devcfg, &m_SpiDevice);
	if(err == ESP_OK)
		isAttached = true;

	return err;
}

void CNRFLib::Begin(nrf_mode_t mode){
	assert(isAttached && "Call AttachToSpiBus first");
	assert(mode == nrf_tx_mode || mode == nrf_rx_mode);

	CELow();
	CSHigh();

	DELAY_MS(5);

	/* Enable each pipes for receiving */
	WriteReg(NRF_REG_ENRXADDR, 0x3F);

	/* Enabled auto acknowledgement */
	WriteReg(NRF_REG_ENAA, 0x3F);

	/* Set payload width to each pipe to 32 bytes */
	for(int i = 0; i < 6; i++)
		WriteReg(NRF_REG_RX_PW_P0, NRF_MAX_PAYLOAD);

	FlushTX();
	FlushRX();

	WriteReg(NRF_REG_SETUP_ADDR_W, 0b11);
	SetAutoRetransmission(10, 5);
	SetChannel(77);
	SetDataRateAndPower(NRF_DATA_RATE_250k, NRF_POWER_0dBm);

	/* Setup config register */
	nrf_reg_config_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.en_crc = NRF_ENABLE_CRC;
	cfg.crco = NRF_CRC_WIDTH;
	cfg.pwr_up = mode == nrf_rx_mode;
	cfg.prim_rx = mode == nrf_rx_mode;
	WriteReg(NRF_REG_CONFIG, _TOUINT(cfg));

	if(mode == nrf_rx_mode)
		CEHigh();

	m_Mode = mode;

	/* Start up delay */
	DELAY_MS(2);

#ifdef DEBUG_MODE
	PrintRegister(GetStatus());
	PrintRegister(GetFifoStatus());

	memset(&cfg, 0, sizeof(cfg));
	uint8_t tmp = ReadReg(NRF_REG_CONFIG);
	memcpy(&cfg, &tmp, sizeof(uint8_t));
	uart << "en_crc: " << cfg.en_crc << " crc: " << cfg.crco << " pwr_up: " << cfg.pwr_up << " prim_rx: " << cfg.prim_rx << endl;

	uint8_t rxAddr[5] = {0};
	uint8_t txAddr[5] = {0};

	ReadBytes(NRF_CMD_R_REGISTER | NRF_REG_RX_ADDR_P0, rxAddr, 5);
	ReadBytes(NRF_CMD_R_REGISTER | NRF_REG_TX_ADDR, txAddr, 5);

	uart << "rx addr pipe 0: ";
	for(int i = 0; i < 5; i++)
		uart << rxAddr[i];
	
	uart << endl << "tx addr: ";
	for(int i = 0; i < 5; i++)
		uart << txAddr[i];
	uart << endl;

	nrf_reg_en_rxaddr_t en;
	memset(&en, 0, sizeof(en));
	tmp = ReadReg(NRF_REG_ENRXADDR);
	memcpy(&en, &tmp, sizeof(uint8_t));
	uart << "pipe 0 enabled: " << en.enrxaddr_pipe0 << endl;
#endif /* DEBUG_MODE */
}

uint8_t CNRFLib::ReadReg(uint8_t reg){
	uint8_t result;
	ReadBytes(NRF_CMD_R_REGISTER | reg, &result, 1);
	return result;
}

uint8_t CNRFLib::WriteReg(uint8_t reg, uint8_t value){
	return WriteBytes(NRF_CMD_W_REGISTER | reg, &value, 1);
}

uint8_t CNRFLib::ReadBytes(uint8_t cmd, uint8_t *pBuff, uint8_t length){
	CSLow();
	ets_delay_us(3);
	uint8_t status = spi_transfer_byte(cmd, m_SpiDevice);
	while(pBuff && length--){
		ets_delay_us(1);
		*pBuff++ = spi_transfer_byte(0xFF, m_SpiDevice);
	}
	ets_delay_us(3);
	CSHigh();

	return status;
}

uint8_t CNRFLib::WriteBytes(uint8_t cmd, uint8_t *pBuff, uint8_t length){
	CSLow();
	ets_delay_us(3);
	uint8_t status = spi_transfer_byte(cmd, m_SpiDevice);
	while(pBuff && length--){
		ets_delay_us(1);
		spi_transfer_byte(*pBuff++, m_SpiDevice);
	}
	ets_delay_us(3);
	CSHigh();

	return status;
}

int8_t CNRFLib::Send(uint8_t *pBuff, uint8_t length){
	assert(isAttached && "Call AttachToSpiBus first");
	assert(m_Mode == nrf_tx_mode && "Tx mode must be set");

	CELow();
	FlushTX();

	/* Setup as tx if needed and wakeup */
	uint8_t tmp = ReadReg(NRF_REG_CONFIG);
	nrf_reg_config_t cfg = *(nrf_reg_config_t*)&tmp;
	cfg.pwr_up = 1;		/* Wakeup */
	cfg.prim_rx = 0;	/* Set tx mode if needed */
	WriteReg(NRF_REG_CONFIG, _TOUINT(cfg));
	DELAY_MS(2); // wakeup delay

	WriteTxPayload(pBuff, length);

	/* Set CE high for 10 us atleast */
	CEHigh();
	ets_delay_us(30);
	CELow();

	/* Wait for success or fail */
	nrf_reg_status_t status;
	do{
		status = GetStatus();
		ets_delay_us(20);
	} while(status.max_rt == 0 && status.tx_ds == 0);

	/* Read retries count */
	nrf_reg_observe_tx_t observe;
	memset(&observe, 0, sizeof(observe));
	tmp = ReadReg(NRF_REG_OBSERVE_TX);
	observe = *(nrf_reg_observe_tx_t*)&tmp;

#ifdef DEBUG_MODE
	PrintRegister(status);
	uart << "Retries count: " << observe.rt_count << " Lost: " << observe.plos_count << endl;
#endif

	/* Set result as -1 on fail, and equal to retries on success */
	uint8_t result;
	if(status.max_rt){
		result = -1;
	} else 
		result = observe.rt_count;

	/* Clear tx_ds and max_rt flags */
	status.rx_dr = 0; // we don't want to clear rx flag
	status.max_rt = 1;
	status.tx_ds = 1;
	WriteReg(NRF_REG_STATUS, _TOUINT(status));

	/* Go to sleep mode */
	SetSleepTxMode();

	return result;
}

void CNRFLib::WriteTxPayload(uint8_t *pBuff, uint8_t length){
	assert(length <= NRF_MAX_PAYLOAD && "Payload width is limited to 32 bytes");

	uint8_t *tmp = (uint8_t*)malloc(NRF_MAX_PAYLOAD);
	memset(tmp, 1, NRF_MAX_PAYLOAD);
	memcpy(tmp, pBuff, length);

	WriteBytes(NRF_CMD_W_TX_PAYLOAD, tmp, NRF_MAX_PAYLOAD);
	free(tmp);
}

void CNRFLib::ReadRxPayload(uint8_t *pBuff, uint8_t length){
	assert(length <= NRF_MAX_PAYLOAD && "Payload width is limited to 32 bytes");

	uint8_t *tmp = (uint8_t*)malloc(NRF_MAX_PAYLOAD);
	ReadBytes(NRF_CMD_R_RX_PAYLOAD, tmp, NRF_MAX_PAYLOAD);
	memcpy(pBuff, tmp, length);
	free(tmp);
}

void CNRFLib::Read(uint8_t *pBuff, uint8_t length){
	assert(isAttached && "Call AttachToSpiBus first");
	assert(m_Mode == nrf_rx_mode && "Rx mode must be set");

	/* Since we are in rx mode we already have CE high, power on and PRX bit */

	ReadRxPayload(pBuff, length);

	/* Clear flags */
	nrf_reg_status_t status = GetStatus();
	status.rx_dr = 1;	// clear
	status.tx_ds = 0;	// don't clear
	status.max_rt = 0;	// don't clear
	WriteReg(NRF_REG_STATUS, _TOUINT(status));
}

bool CNRFLib::IsRxDataAvailable(uint8_t pipeNo){
	nrf_reg_status_t status = GetStatus();
	return status.rx_p_n == pipeNo;
}

bool CNRFLib::IsRxDataAvailable(){
	nrf_reg_status_t status = GetStatus();
	return status.rx_dr;
}

void CNRFLib::SetSleepTxMode(){
	uint8_t tmp = ReadReg(NRF_REG_CONFIG);
	nrf_reg_config_t cfg = *(nrf_reg_config_t*)&tmp;
	cfg.pwr_up = 0;
	cfg.prim_rx = 0;	/* Actually rx mode */
	WriteReg(NRF_REG_CONFIG, _TOUINT(cfg));
	CELow();

	m_Mode = nrf_tx_mode;
}
void CNRFLib::SetRxMode(){
	uint8_t tmp = ReadReg(NRF_REG_CONFIG);
	nrf_reg_config_t cfg = *(nrf_reg_config_t*)&tmp;
	cfg.pwr_up = 1;
	cfg.prim_rx = 1;	/* Actually rx mode */
	WriteReg(NRF_REG_CONFIG, _TOUINT(cfg));
	CEHigh();

	m_Mode = nrf_rx_mode;
	DELAY_MS(2);
}

void CNRFLib::FlushRX(){
	WriteBytes(NRF_CMD_FLUSH_RX, NULL, 0);
}

void CNRFLib::FlushTX(){
	WriteBytes(NRF_CMD_FLUSH_TX, NULL, 0);
}

nrf_reg_status_t CNRFLib::GetStatus(){
	uint8_t tmp = ReadReg(NRF_REG_STATUS);
	nrf_reg_status_t status;
	memcpy((void*)&status, &tmp, sizeof(tmp));
	return status;
}

nrf_reg_fifo_status_t CNRFLib::GetFifoStatus(){
	uint8_t tmp = ReadReg(NRF_REG_FIFO_STATUS);
	nrf_reg_fifo_status_t status;
	memcpy((void*)&status, &tmp, sizeof(tmp));
	return status;
}

void CNRFLib::SetAutoRetransmission(uint8_t count, uint8_t delay){
	assert(count <= 15 && delay <= 15);

	nrf_reg_retries_t rt;
	rt.count = count;
	rt.delay = delay;
	WriteReg(NRF_REG_SETUP_RETR, _TOUINT(rt));
}

void CNRFLib::SetChannel(uint8_t channel){
	assert(channel < 0x80 && "Note that ESP32 is little-endian chip");

	WriteReg(NRF_REG_SETUP_CHANNEL, channel);
}

void CNRFLib::SetDataRate(uint8_t dataRate){
	assert(dataRate < 3);

	uint8_t tmp = ReadReg(NRF_REG_SETUP);
	nrf_reg_setup_t setup = *(nrf_reg_setup_t*)&tmp;
	setup.dr_high = (dataRate >> 1) & 1;
	setup.dr_low = dataRate & 1;

#ifdef DEBUG_MODE
	uart << "High: " << ((dataRate >> 1) & 1) << " Low: " << (dataRate & 1) << endl;
#endif

	WriteReg(NRF_REG_SETUP, _TOUINT(setup));
}

void CNRFLib::SetPower(uint8_t power){
	assert(power <= 3);

	uint8_t tmp = ReadReg(NRF_REG_SETUP);
	nrf_reg_setup_t setup = *(nrf_reg_setup_t*)&tmp;
	setup.power = power;
	WriteReg(NRF_REG_SETUP, _TOUINT(setup));
}

void CNRFLib::SetDataRateAndPower(uint8_t dataRate, uint8_t power){
	assert(dataRate < 3);
	assert(power <= 3);

	uint8_t tmp = ReadReg(NRF_REG_SETUP);
	nrf_reg_setup_t setup = *(nrf_reg_setup_t*)&tmp;
	setup.dr_high = (dataRate >> 1) & 1;
	setup.dr_low = dataRate & 1;
	setup.power = power;
	WriteReg(NRF_REG_SETUP, _TOUINT(setup));
}

void CNRFLib::SetPipeAddr(uint8_t pipeNo, uint8_t *pAddr, uint8_t length){
	assert(pipeNo < 6);
	assert(length == (pipeNo < 2 ? 5 : 1));

	WriteBytes((NRF_CMD_W_REGISTER | NRF_REG_RX_ADDR_P0) + pipeNo, pAddr, length);
}

void CNRFLib::GetPipeAddr(uint8_t pipeNo, uint8_t *pAddr){
	assert(pipeNo < 6);

	uint8_t length = pipeNo < 2 ? 5 : 1;
	ReadBytes((NRF_CMD_R_REGISTER | NRF_REG_RX_ADDR_P0) + pipeNo, pAddr, length);
}

void CNRFLib::SetTxAddr(uint8_t *pAddr, uint8_t length){
	assert(length == 5);

	WriteBytes(NRF_CMD_W_REGISTER | NRF_REG_TX_ADDR, pAddr, length);
}
void CNRFLib::GetTxAddr(uint8_t *pAddr){
	WriteBytes(NRF_CMD_R_REGISTER | NRF_REG_TX_ADDR, pAddr, 5);
}

void CNRFLib::ScanChannels(uint64_t &firstHalf, uint64_t &secondHalf){
	nrf_mode_t wasMode = m_Mode;
	if(wasMode == nrf_tx_mode)
		SetRxMode();

	for(int i = 0; i < 64; i++){
		SetChannel(i);
		uint8_t value = ReadReg(NRF_REG_RPD);
		firstHalf |= (value << i);
#ifdef DEBUG_MODE
		uart << "Channel: " << i << " value: " << value << endl;
#endif
	}

	for(int i = 0; i < 64; i++){
		SetChannel(64+i);
		uint8_t value = ReadReg(NRF_REG_RPD);
		secondHalf |= (value << i);
	}

	if(wasMode == nrf_tx_mode)
		SetSleepTxMode();
}

void CNRFLib::CELow(){
	gpio_set_direction(m_CE, GPIO_MODE_OUTPUT);
	gpio_set_level(m_CE, 0);
}
void CNRFLib::CEHigh(){
	gpio_set_direction(m_CE, GPIO_MODE_OUTPUT);
	gpio_set_level(m_CE, 1);
}

void CNRFLib::CSLow(){
	gpio_set_direction(m_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(m_CS, 0);
}
void CNRFLib::CSHigh(){
	gpio_set_direction(m_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(m_CS, 1);
}

#ifdef DEBUG_MODE
void CNRFLib::PrintRegister(nrf_reg_config_t config){
	uart << "rx_dr: " << config.mask_rx_dr << " tx_ds: " << config.mask_tx_ds << " max_rt: " << config.mask_max_rt << " "
		<< "en_crc: " << config.en_crc << " crco: " << config.crco << " power_up: " << config.pwr_up << " prx: " << config.prim_rx << endl;
}

void CNRFLib::PrintRegister(nrf_reg_status_t status){
	uart << "tx_full: " << status.tx_full << " rx_p_no: " << status.rx_p_n << " max_rt: " << status.max_rt
		<< " tx_ds: " << status.tx_ds << " rx_dr: " << status.rx_dr << endl;
}

void CNRFLib::PrintRegister(nrf_reg_fifo_status_t status){
	uart << "rx_empty: " << status.rx_empty << " rx_full: " << status.rx_full << " tx_empty: " 
		<< status.tx_empty << " tx_full: " << status.tx_full << " tx_reuse: " << status.tx_reuse << endl;
}
#endif /* DEBUG_MODE */
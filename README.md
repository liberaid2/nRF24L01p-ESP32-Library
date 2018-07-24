# nRF24L01p Library for ESP32 (esp-idf)
## Simple usage example
```cpp
/* Sender */
CNRFLib nrf(GPIO_NUM_16, GPIO_NUM_17);
nrf.AttachToSpiBus(HSPI_HOST);
uint8_t buff[32] = {0};
nrf.Begin(nrf_tx_mode);
nrf.Send(buff, 32);

.....................................

/* Receiver */
CNRFLib nrf(GPIO_NUM_16, GPIO_NUM_17);
nrf.AttachToSpiBus(HSPI_HOST);
uint8_t buff[32] = {0};
nrf.Begin(nrf_rx_mode);
while(!nrf.IsRxDataAvailable())
    ;
nrf.Read(buff, 32);
```

## Full usage example
```cpp
/* Include library */
#include "nrf24l01p_lib.h"

/* Include FreeRTOS for delays */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Default SPI pins for HSPI */
#define SPI_SCLK 14
#define SPI_MISO 12
#define SPI_MOSI 13

/* How to determine sender and receiver */
#define NRF_MODE_SENDER 1
#define NRF_MODE_RECEIVER 2

/* Set your mode here */
#define NRF_MODE NRF_MODE_SENDER

void NrfTest(){
    	/* Init spi bus */
    	spi_bus_config_t buscfg;
	memset(&buscfg, 0, sizeof(buscfg));

	buscfg.miso_io_num = SPI_MISO;
	buscfg.mosi_io_num = SPI_MOSI;
	buscfg.sclk_io_num = SPI_SCLK;
	buscfg.quadhd_io_num = -1;
	buscfg.quadwp_io_num = -1;
	buscfg.max_transfer_sz = 4096;

	esp_err_t err = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	assert(err == ESP_OK);
	
	/* Create instance of CNRFLib */
	CNRFLib nrf(GPIO_NUM_16, GPIO_NUM_17);
	/* Attach to spi bus */
	nrf.AttachToSpiBus(HSPI_HOST);
	
	/* Buffer for tx/rx operations */
	uint8_t buff[32] = {0};
	/* Setup custom address */
	uint8_t addr[5] = {222, 111, 001, 100, 040};
	
	

	if(NRF_MODE == NRF_MODE_SENDER){
		printf("Tx mode\n");
		
		/* Fill buffer with dummy content */
		for(int i = 0; i < 32; i++)
			buff[i] = i;

        	/* Init library with Begin() */
		nrf.Begin(nrf_tx_mode);
		
		/* Set addr to which we will send data */
		nrf.SetTxAddr(addr, 5);
		/* Set pip0 addr to the same value to receive acks */
		nrf.SetPipeAddr(0, addr, 5);
		
		int8_t result;
		while(1){
		    /* Check out Send function description */
			result = nrf.Send(buff, 32);
			buff[1]++;
            
            printf("Result: %d\n", result);
			vTaskDelay(pdMS_TO_TICKS(500));
		}
	} else {
		printf("Rx mode\n");

        	/* Init library */
		nrf.Begin(nrf_rx_mode);
		/* Set pipe0 addr to listen to packets with this addr */
		nrf.SetPipeAddr(0, addr, 5);

		while(1){
			vTaskDelay(pdMS_TO_TICKS(1));

            		/* Check for available packets in rx buffer */
			if(!nrf.IsRxDataAvailable())
				continue;

            		/* Read it */
			nrf.Read(buff, 32);
			printf("Received: ");
			for(int i = 0; i < 32; i++)
			    printf("%d ", buff[i]);
			printf("\n");
		}
	}
}
```

## Limitations
* Unable to enable or disable specific pipe
* Unable to enable or disable auto acknowledgment for specific pipe
* Unable to change payload width for specific pipe
* There is no support for dynamic payload width
* There is no support for interrupts
* Unable to set address width different from 5 btyes
* Unable to dynamically change crc values

# License
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

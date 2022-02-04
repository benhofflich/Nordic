/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup fatfs_example_main main.c
 * @{
 * @ingroup fatfs_example
 * @brief FATFS Example Application main file.
 *
 * This file contains the source code for a sample application using FAT filesystem and SD card library.
 *
 */


#include "nrf.h"
//#include "bsp.h"
#include "boards.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_pdm.h"
#define _pin_clk 5
#define _pin_din 6


#define FILE_NAME   "NORDIC.pcm"
#define TEST_STRING "SD card example."


#define LED1 NRF_GPIO_PIN_MAP(0,28)
int16_t buff1[1024];
int16_t buff2[1024];
bool flag =0;
bool WriteFlag = 0;
int32_t interupt = 0;
int32_t pisanje = 0;

static void drv_pdm_hand(const nrfx_pdm_evt_t *evt){
	//interupt++;
	//if((*evt).error != 0){int hhh = 1;} //(*evt).error, evt->error
	
	if((*evt).buffer_requested){
		if(!flag){
	//memcpy(&buff2[0], (*evt).buffer_released, 1000);
			int32_t error = nrfx_pdm_buffer_set(&buff1[0], 1024);
			flag = 1;
			WriteFlag = 1;}
		else{
			int32_t error = nrfx_pdm_buffer_set(&buff2[0], 1024);
			flag = 0;
			WriteFlag = 1;
		}
		
	
}
}
/**
 * @brief Function for main application entry.
 */
int main(void)
{
		//nrf_gpio_cfg_output(LED1); 
    //bsp_board_init(BSP_INIT_LEDS);
		

    //APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    //NRF_LOG_DEFAULT_BACKENDS_INIT();

    //NRF_LOG_INFO("FATFS example started.");
		uint32_t error = 0;
		nrfx_pdm_config_t config1 = NRFX_PDM_DEFAULT_CONFIG(_pin_clk, _pin_din);
		error = nrfx_pdm_init(&config1, drv_pdm_hand);
		error = nrfx_pdm_start();
		
		//if(error != 0){int hhh = 1;}
		//
    while (true)
    {	while(!WriteFlag);
//			if(error != 0){int hhhj =1;}
			
		
    }
}

/** @} */

/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
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
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrfx_pdm.h"
#include "nrf_pdm.h"
#include "nrfx_gpiote.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_nvmc.h"
#include "nrf_cli.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define _pin_clk NRF_GPIO_PIN_MAP(0,5)
#define _pin_din NRF_GPIO_PIN_MAP(0,6)

uint16_t buffsize = 2048;
int16_t buff1[2048];
int16_t buff2[2048];
bool flag = 0;
bool writeFlag = 0;
uint32_t page = 0x00080000;

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x80000,
    .end_addr   = 0xFFFFF,
};


//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 1024                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1024                         /**< UART RX buffer size. */

#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

static void drv_pdm_hand(const nrfx_pdm_evt_t *evt){

  nrfx_err_t rc = 0;
  if((*evt).buffer_released){
    if(writeFlag){
      if(!flag){
        rc = nrf_fstorage_write(&fstorage, page, buff2, sizeof(buff2), NULL);
        APP_ERROR_CHECK(rc);
        writeFlag = 0;
      }
      else{
        rc = nrf_fstorage_write(&fstorage, page, buff1, sizeof(buff1), NULL);
        APP_ERROR_CHECK(rc);
        writeFlag = 0;
      }
      page += 4096;
    }
  }
  if((*evt).buffer_requested){
    if(!flag){
      rc = nrfx_pdm_buffer_set(buff1, buffsize);
      flag = 1;
      writeFlag = 1;
      //error = nrfx_pdm_start();
    }
    else{
      rc = nrfx_pdm_buffer_set(buff2, buffsize);
      flag = 0;
      writeFlag = 1;
      //error = nrfx_pdm_start();
    }
  }
}

static void audio_init()
{
  ret_code_t err;
  nrfx_pdm_config_t config1 = NRFX_PDM_DEFAULT_CONFIG(_pin_clk, _pin_din);

  
  nrf_pdm_enable();
  err = nrfx_pdm_init(&config1, drv_pdm_hand);
  APP_ERROR_CHECK(err);

  err = nrfx_pdm_start();
  APP_ERROR_CHECK(err);

}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t rc;
    log_init();
    NRF_LOG_INFO("PDM STARTED");

    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_nvmc;
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    audio_init();


    // This part of the example is just for testing the loopback .
    while (true)
    {
    //NRF_LOG_FLUSH();
      /*for(size_t i = 0; i < buffsize; i++){
        NRF_LOG_INFO("%d",buff1[i]);
        NRF_LOG_FLUSH();
      }*/
      
    } 
}


/** @} */

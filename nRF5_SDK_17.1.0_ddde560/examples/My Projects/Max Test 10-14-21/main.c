/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

#define address  (0xAA >>1)
#define accelAddress 0x1F

#define SUCCESS 0x00

#define CMD_DELAY 6
#define CMD_ENABLE_DELAY 45

#define numTries 50

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// A flag to indicate the transfer state
static volatile bool m_xfer_done = false;

static uint8_t response[3];
static uint8_t responseSize = 3;
static uint8_t numSamples;
static uint16_t dataSize;
uint8_t * samples;
uint8_t * accelData;
static bool accelPresent = true;
static uint8_t sampleMode;
/*Read Samples Mode:
  mode == 0 --> Calibration Mode
  mode == 1 --> Raw Data Mode
  mode == 2 --> HR/SpO2 Algorithm Mode
*/

uint16_t heartRate;
uint8_t confidence;
uint16_t oxygen;
uint8_t sensorStatus;
uint32_t infrared;
uint32_t redCounter;

void data_handler(uint8_t * response){
  NRF_LOG_HEXDUMP_DEBUG(response,responseSize);
}

//Event Handler
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
        //If data transmission or receiving is finished
	case NRF_DRV_TWI_EVT_DONE:
        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
        {
            //data_handler(response);
        }
        m_xfer_done = true;//Set the flag
        break;
        
        default:
        // do nothing
          break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

uint8_t writeByte(uint8_t * reg, uint8_t cmdLength)
{
  uint8_t statusByte = 0;
  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, address, reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_DELAY);

  m_xfer_done = false;

  for(size_t i = 0; i < numTries; i++)
  {
    err_code = nrf_drv_twi_rx(&m_twi, address, &statusByte, sizeof(statusByte));
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);
    if(!statusByte){break;}
    nrf_delay_ms(CMD_DELAY);
  }
  return statusByte;
}

void writeAccel(uint8_t * reg, uint8_t cmdLength)
{
  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, accelAddress, reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_DELAY);
}

uint8_t writeEnableByte(uint8_t * reg, uint8_t cmdLength)
{
  uint8_t statusByte = 0;
  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, address, reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_ENABLE_DELAY);

  m_xfer_done = false;

  for(size_t i = 0; i < numTries; i++)
  {
    err_code = nrf_drv_twi_rx(&m_twi, address, &statusByte, sizeof(statusByte));
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);
    if(!statusByte){break;}
    nrf_delay_ms(CMD_ENABLE_DELAY);
  }
  return statusByte;
}

uint8_t readByte(uint8_t * reg, uint8_t cmdLength)
{
  uint8_t readBuf[2];
  uint8_t statusByte;
  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, address, reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_DELAY);

  m_xfer_done = false;

  for(size_t i = 0; i < numTries; i++)
  {
    err_code = nrf_drv_twi_rx(&m_twi, address, readBuf, sizeof(readBuf));
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);
    statusByte = readBuf[0];
    if(!statusByte){break;}
    nrf_delay_ms(CMD_DELAY);
  }

  if(statusByte){
    return statusByte;
  }

  uint8_t returnByte = readBuf[1];
  return returnByte;
}

uint8_t readFillArray(uint8_t * reg, uint8_t cmdLength, uint8_t readBytes, uint8_t * data)
{
  uint8_t statusByte;
  uint8_t fillArraySize = (uint8_t)(readBytes + sizeof(statusByte));
  uint8_t fillArray[fillArraySize];

  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, address, reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_DELAY);

  m_xfer_done = false;

 for(size_t i = 0; i < numTries; i++)
  {
    err_code = nrf_drv_twi_rx(&m_twi, address, fillArray, sizeof(fillArray));
    APP_ERROR_CHECK(err_code);

    while (m_xfer_done == false);
    uint8_t statusByte = fillArray[0];
    if(!statusByte){break;}
    nrf_delay_ms(CMD_DELAY);
  }

  if(statusByte){// SUCCESS: 0x00
    for(size_t i = 0; i < readBytes; i++){
      data[i] = 0;
    }
    return statusByte;
  }

  for(size_t i = 0; i < readBytes; i++){
    data[i] = fillArray[i+1];
  }
  return statusByte;
}

void readFillAccelArray(uint8_t reg, uint8_t cmdLength, uint8_t readBytes, uint8_t * data)
{
  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, accelAddress, &reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_DELAY);

  m_xfer_done = false;

  err_code = nrf_drv_twi_rx(&m_twi, accelAddress, data, readBytes);
  APP_ERROR_CHECK(err_code);

  while (m_xfer_done == false);
  nrf_delay_ms(CMD_DELAY);

}

void accelInit(void)
{
  NRF_LOG_DEBUG("Initializing Accelerometer");
  NRF_LOG_FLUSH();
  uint8_t buf[2] = {0x1B, 0xE0};
  writeAccel(buf, sizeof(buf));
  uint8_t buf2[2] = {0x17, 0x10};
  writeAccel(buf2, sizeof(buf2));
}

void input_pin_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  //Do Nothing
}

void setMode(uint8_t mode)
{
    ret_code_t err_code;
    uint8_t _resetPin = 7;
    uint8_t _mfioPin = 8;

    err_code = nrf_drv_gpiote_init(); // Initialize the GPIOTE
    APP_ERROR_CHECK(err_code); // check for the errors

    nrf_gpio_cfg_output(_mfioPin);
    nrf_gpio_cfg_output(_resetPin);

    //Application Mode
    if(mode == 1){
      nrf_gpio_pin_clear(_mfioPin);
    } else {
      nrf_gpio_pin_set(_mfioPin);
    }
    nrf_gpio_pin_clear(_resetPin);
    nrf_delay_ms(10);
    nrf_gpio_pin_set(_resetPin);
    if(mode == 1){
      nrf_delay_ms(50);
    }else{
      nrf_delay_ms(1000);
      nrf_drv_gpiote_in_config_t  in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true); // Falling edge interrupt detection
      in_config.pull = NRF_GPIO_PIN_PULLUP;

      err_code = nrf_drv_gpiote_in_init(_mfioPin, &in_config, NULL); // Initialize the interrupt pin 
      APP_ERROR_CHECK(err_code);

      nrf_drv_gpiote_in_event_enable(_mfioPin, true); // Enable the interrupt events
    }
}

void readHubStatus(void)
{
  NRF_LOG_DEBUG("READ HUB STATUS");
  NRF_LOG_FLUSH();
  uint8_t buf[2] = {0x02, 0x00};
  uint8_t status = readByte(buf, sizeof(buf));
  if(status == 0x08){
    NRF_LOG_INFO("Sensor in Bootloader Mode");
  } else {
    NRF_LOG_INFO("Sensor in Operating mode");
  }
}

void setOutputMode(uint8_t mode)
{
  NRF_LOG_DEBUG("SET OUTPUT MODE");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x10, 0x00, mode};
  uint8_t status = writeByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void setHubThreshold(uint8_t threshold)
{
  NRF_LOG_DEBUG("SET HUB THRESHOLD");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x10, 0x01, threshold};
  uint8_t status = writeByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void enableAGC(void)
{
  NRF_LOG_DEBUG("ENABLE AGC");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x52, 0x00, 0x01};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void disableAGC(void)
{
  NRF_LOG_DEBUG("DISABLE AGC");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x52, 0x00, 0x00};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void enableAccel(void)
{
  NRF_LOG_DEBUG("ENABLE ACCEL");
  NRF_LOG_FLUSH();
  uint8_t buf[4] = {0x44, 0x04, 0x01, 0x01};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void disableAccel(void)
{
  NRF_LOG_DEBUG("DISABLE ACCEL");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x44, 0x04, 0x00};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void enableMAX30101(void)
{
  NRF_LOG_DEBUG("ENABLE MAX30101");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x44, 0x03, 0x01};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void disableMAX30101(void)
{
  NRF_LOG_DEBUG("DISABLE MAX30101");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x44, 0x03, 0x00};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void enableAlgorithm(uint8_t alg)
{
  NRF_LOG_DEBUG("ENABLE ALGORITHM");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x52, alg, 0x01};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void disableAlgorithm(uint8_t alg)
{
  NRF_LOG_DEBUG("DISABLE ALGORITHM");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x52, alg, 0x00};
  uint8_t status = writeEnableByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void adjustLED1(uint8_t scale)
{
  NRF_LOG_DEBUG("ADJUST LED1");
  NRF_LOG_FLUSH();
  uint8_t buf[4] = {0x40, 0x03, 0x0C, scale};
  uint8_t status = writeByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void adjustLED2(uint8_t scale)
{
  NRF_LOG_DEBUG("ADJUST LED2");
  NRF_LOG_FLUSH();
  uint8_t buf[4] = {0x40, 0x03, 0x0D, scale};
  uint8_t status = writeByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void setCalibrationCoef(void)
{
  NRF_LOG_DEBUG("SET CALIBRATION COEFFICIENTS");
  NRF_LOG_FLUSH();
  uint8_t buf[15] = {0x50, 0x02, 0x0B,
                     0x00, 0x02, 0x6F, 0x60, //A = 1.5958422
                     0xFF, 0xCB, 0x1D, 0x12, //B = -34.659664
                     0x00, 0xAB, 0xF3, 0x7B};//C = 112.68987
  uint8_t status = writeByte(buf, sizeof(buf));
  if(status){
    NRF_LOG_DEBUG("WRITE FAILED. ERROR CODE: %d",status);
  }
}

void startAlgorithm(uint8_t outputMode, uint8_t alg)
{
  setOutputMode(outputMode);
  NRF_LOG_FLUSH();
  setHubThreshold(0x01);

  if(sampleMode != 1){
    enableAGC();
  } 

  enableMAX30101();

  if(accelPresent){
    //enableAccel();
    accelInit();
  }

  enableAlgorithm(alg);

  if(sampleMode == 1){
    disableAGC();
  } 

  nrf_delay_ms(1000);
  
  if(sampleMode == 1){
    adjustLED1(0x7F);
    adjustLED2(0x7F);
  }
}

uint8_t readHubStatusByte(void)
{
  NRF_LOG_DEBUG("READING SENSOR HUB STATUS BYTE");
  NRF_LOG_FLUSH();
  uint8_t buf[2] = {0x00, 0x00};
  uint8_t statusByte = readByte(buf, sizeof(buf));
  return statusByte;
}

uint8_t getNumSamples(void)
{
  NRF_LOG_DEBUG("READING NUMBER OF SAMPLES");
  NRF_LOG_FLUSH();
  uint8_t buf[2] = {0x12, 0x00};
  uint8_t numberofSamples = readByte(buf, sizeof(buf));
  return numberofSamples;
}

uint8_t getNumAccSamples(void)
{
  NRF_LOG_DEBUG("READING NUMBER OF ACCELEROMETER SAMPLES");
  NRF_LOG_FLUSH();
  uint8_t buf[3] = {0x13, 0x00, 0x04};
  uint8_t numberofSamples = readByte(buf, sizeof(buf));
  return numberofSamples;
}

void readAccelSamples(void)
{
  int16_t pACC_X;
  int16_t pACC_Y;
  int16_t pACC_Z;
  uint8_t accSampleLength = 6;
  uint8_t accSamples[accSampleLength];
  accelData = accSamples;

  uint8_t accelReg = 0x08;
  readFillAccelArray(accelReg, 1, accSampleLength, accelData);

  pACC_X = ((uint16_t)(accelData[1] << 8)) | (uint16_t)accelData[0];
  if(pACC_X & 0x8000){ pACC_X-=65536;}

  pACC_Y = ((uint16_t)(accelData[3] << 8)) | (uint16_t)accelData[2];
  if(pACC_Y & 0x8000){ pACC_Y-=65536;}

  pACC_Z = ((uint16_t)(accelData[5] << 8)) | (uint16_t)accelData[4];
  if(pACC_Z & 0x8000){ pACC_Z-=65536;}

  NRF_LOG_INFO("ACCELEROMETER DATA: X = %d, Y = %d, Z = %d",pACC_X, pACC_Y, pACC_Z);
  NRF_LOG_FLUSH();
}

void readSamples(void)
{
  uint8_t sampleLength;

  switch(sampleMode){
    case 0:
      sampleLength = 32;
      break;
    case 1:
      sampleLength = 12;
      break;
    case 2:
      sampleLength = 6;
      break;
  }

  uint8_t sampleArray[sampleLength];
  samples = sampleArray;

  uint8_t statusByte = readHubStatusByte();
  if(statusByte == 1)
  {
    return;
  }
  getNumSamples();

  NRF_LOG_DEBUG("READING SAMPLES");
  NRF_LOG_FLUSH();
  uint8_t fifoReg[2] = {0x12, 0x01};
  uint8_t status = readFillArray(fifoReg, sizeof(fifoReg), sampleLength, samples);

  NRF_LOG_HEXDUMP_DEBUG(samples,sampleLength);
  NRF_LOG_FLUSH();

  readAccelSamples();
  if(sampleMode == 1){
    infrared = ((uint32_t)(samples[0]<<16))|(uint32_t)(samples[1]<<8)|(samples[2]);
    redCounter = ((uint32_t)(samples[3]<<16))|(uint32_t)(samples[4]<<8)|(samples[5]);
    NRF_LOG_INFO("Infrared: %d",infrared);
    NRF_LOG_INFO("Red Counter: %d",redCounter);
  }
  
  if(sampleMode == 2){
    heartRate = ((uint16_t)(samples[0]<<8))|(samples[1]);
    heartRate /= 10;
    confidence = samples[2];
    oxygen = ((uint16_t)(samples[3]<<8))|(samples[4]);
    oxygen /= 10;
    sensorStatus = samples[5];

    NRF_LOG_INFO("Heartrate: %d",heartRate);
    NRF_LOG_INFO("Confidence: %d",confidence);
    NRF_LOG_INFO("Oxygen: %d",oxygen);
    NRF_LOG_INFO("Status: %d",sensorStatus);
  }
  NRF_LOG_FLUSH();
}

void stopAlgorithm(alg)
{
  disableMAX30101();
  if(accelPresent){
    disableAccel();
  }
  disableAlgorithm(alg);
}

void calibrate(void)
{
  NRF_LOG_DEBUG("CALIBRATION");
  NRF_LOG_FLUSH();
  sampleMode = 0;
  startAlgorithm(0x03,0x02);
  readSamples();
  stopAlgorithm();
}

void rawData(void)
{
  uint8_t alg = 0x01;
  NRF_LOG_DEBUG("RAW DATA COLLECTION");
  NRF_LOG_FLUSH();
  sampleMode = 1;
  startAlgorithm(0x01,alg);
  nrf_delay_ms(4000);
  while(true){
    readSamples();
    nrf_delay_ms(250);
  }
  stopAlgorithm(alg);
}

void hrSpO2Algorithm(void)
{
  uint8_t alg = 0x01;
  NRF_LOG_DEBUG("HEARTRATE/SPO2 ALGORITHM");
  NRF_LOG_FLUSH();
  sampleMode = 2;
  startAlgorithm(0x02,alg);
  nrf_delay_ms(4000);
  while(true){
    readSamples();
    nrf_delay_ms(250);
  }
  stopAlgorithm(alg);
}

void bpAlgorithm(void)
{
  uint8_t alg = 0x01;
  NRF_LOG_DEBUG("HEARTRATE/SPO2 ALGORITHM");
  NRF_LOG_FLUSH();
  sampleMode = 2;
  startAlgorithm(0x02,alg);
  nrf_delay_ms(4000);
  while(true){
    readSamples();
    nrf_delay_ms(250);
  }
  stopAlgorithm(alg);
}

int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nSensor started.");
    NRF_LOG_FLUSH();
    twi_init();
    setMode(0);

    //readHubStatus();
    //calibrate();
    rawData();
    //hrSpO2Algorithm();

    while (true)
    {
    //hrSpO2Algorithm();

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        NRF_LOG_FLUSH();
    }
}
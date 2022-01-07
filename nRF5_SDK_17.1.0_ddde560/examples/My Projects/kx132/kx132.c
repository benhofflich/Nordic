
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "kx132.h"
#include "nrf_delay.h"




//Initializing TWI0 instance
#define TWI_INSTANCE_ID     0

// A flag to indicate the transfer state
static volatile bool m_xfer_done = false;


// Create a Handle for the twi communication
nrf_drv_twi_t m_twi;

void writeAccel(uint8_t * reg, uint8_t cmdLength)
{
  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, kx132_ADDRESS, reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_DELAY);
}




void readFillAccelArray(uint8_t reg, uint8_t cmdLength, uint8_t readBytes, uint8_t * data)
{
  ret_code_t err_code;

  m_xfer_done = false;

  err_code = nrf_drv_twi_tx(&m_twi, kx132_ADDRESS, &reg, cmdLength, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false);

  nrf_delay_ms(CMD_DELAY);

  m_xfer_done = false;

  err_code = nrf_drv_twi_rx(&m_twi, kx132_ADDRESS, data, readBytes);
  APP_ERROR_CHECK(err_code);

  while (m_xfer_done == false);
  nrf_delay_ms(CMD_DELAY);

}


/*
  Function to initialize the kx132
*/ 
void accelInit(void)
{
  uint8_t buf[2] = {kx132_CNTL1, 0xE0};
  writeAccel(buf, sizeof(buf));
  uint8_t buf2[2] = {kx132_INS2, 0x10};
  writeAccel(buf2, sizeof(buf2));
}



/*
  A Function to read accelerometer's values from the internal registers of kx132
*/ 
void readAccelSamples(uint16_t * pACC_X, uint16_t * pACC_Y, uint16_t * pACC_Z)
{
  uint8_t accSampleLength = 6;
  uint8_t accSamples[accSampleLength];

  uint8_t accelReg = 0x08;
  readFillAccelArray(accelReg, 1, accSampleLength, accSamples);

  *pACC_X = ((uint16_t)(accSamples[1] << 8)) | (uint16_t)accSamples[0];
  if(*pACC_X & 0x8000){ *pACC_X-=65536;}

  *pACC_Y = ((uint16_t)(accSamples[3] << 8)) | (uint16_t)accSamples[2];
  if(*pACC_Y & 0x8000){ *pACC_Y-=65536;}

  *pACC_Z = ((uint16_t)(accSamples[5] << 8)) | (uint16_t)accSamples[4];
  if(*pACC_Z & 0x8000){ *pACC_Z-=65536;}
}






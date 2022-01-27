
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "kx132.h"




//Initializing TWI0 instance
#define TWI_INSTANCE_ID     0

// A flag to indicate the transfer state
static volatile bool m_xfer_done = false;


// Create a Handle for the twi communication
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);




//Event Handler
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
        //If data transmission or receiving is finished
	case NRF_DRV_TWI_EVT_DONE:
        m_xfer_done = true;//Set the flag
        break;
        
        default:
        // do nothing
          break;
    }
}



//Initialize the TWI as Master device
void twi_master_init(void)
{
    ret_code_t err_code;

    // Configure the settings for twi communication
    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,  //SCL Pin
       .sda                = TWI_SDA_M,  //SDA Pin
       .frequency          = NRF_DRV_TWI_FREQ_400K, //Communication Speed
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //Interrupt Priority(Note: if using Bluetooth then select priority carefully)
       .clear_bus_init     = false //automatically clear bus
    };


    //A function to initialize the twi communication
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    //Enable the TWI Communication
    nrf_drv_twi_enable(&m_twi);
}



/*
   A function to write a Single Byte to kx132's internal Register
*/ 
bool kx132_register_write(uint8_t register_address, uint8_t value)
{
    ret_code_t err_code;
    uint8_t tx_buf[kx132_ADDRESS_LEN+1];
	
    //Write the register address and data into transmit buffer
    tx_buf[0] = register_address;
    tx_buf[1] = value;

    //Set the flag to false to show the transmission is not yet completed
    m_xfer_done = false;
    
    //Transmit the data over TWI Bus
    err_code = nrf_drv_twi_tx(&m_twi, kx132_ADDRESS, tx_buf, kx132_ADDRESS_LEN+1, false);
    
    //Wait until the transmission of the data is finished
    while (m_xfer_done == false)
    {
      }

    // if there is no error then return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;	
}




/*
  A Function to read data from the kx132
*/ 
bool kx132_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    ret_code_t err_code;

    //Set the flag to false to show the receiving is not yet completed
    m_xfer_done = false;
    
    // Send the Register address where we want to write the data
    err_code = nrf_drv_twi_tx(&m_twi, kx132_ADDRESS, &register_address, 1, true);
	  
    //Wait for the transmission to get completed
    while (m_xfer_done == false){}
    
    // If transmission was not successful, exit the function with false as return value
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    //set the flag again so that we can read data from the kx132's internal register
    m_xfer_done = false;
	  
    // Receive the data from the kx132
    err_code = nrf_drv_twi_rx(&m_twi, kx132_ADDRESS, destination, number_of_bytes);
		
    //wait until the transmission is completed
    while (m_xfer_done == false){}
	
    // if data was successfully read, return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;
}



/*
  A Function to verify the product id
  (its a basic test to check if we are communicating with the right slave, every type of I2C Device has 
  a special WHO_AM_I register which holds a specific value, we can read it from the kx132 or any device
  to confirm we are communicating with the right device)
*/ 
bool kx132_verify_product_id(void)
{
    uint8_t who_am_i; // create a variable to hold the who am i value


    // Note: All the register addresses including WHO_AM_I are declared in 
    // kx132.h file, you can check these addresses and values from the
    // datasheet of your slave device.
    if (kx132_register_read(WHO_AM_I, &who_am_i, 1))
    {
        if (who_am_i != kx132_WHO_AM_I)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}


/*
  Function to initialize the kx132
*/ 
bool kx132_init(void)
{   
  bool transfer_succeeded = true;
	
  //Check the id to confirm that we are communicating with the right device
  transfer_succeeded &= kx132_verify_product_id();
	
  if(kx132_verify_product_id() == false)
    {
	return false;
      } 		

  (void)kx132_register_write(kx132_CNTL1 , 0xE0);
  (void)kx132_register_write(kx132_INS2 , 0x10);

  return transfer_succeeded;
}





/*
  A Function to read accelerometer's values from the internal registers of kx132
*/ 
bool kx132_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z )
{

  uint8_t buf[6];
  uint8_t i;
  bool ret = false;		
  
  
  if(kx132_register_read(kx132_XOUT_L, buf, 6) == true)
  {
    kx132_register_read(kx132_XOUT_L, buf, 6);
    *pACC_X = (buf[1] << 8) | buf[0];
    if(*pACC_X & 0x8000) *pACC_X-=65536;

    *pACC_Y= (buf[3] << 8) | buf[2];
    if(*pACC_Y & 0x8000) *pACC_Y-=65536;

    *pACC_Z = (buf[5] << 8) | buf[4];
    if(*pACC_Z & 0x8000) *pACC_Z-=65536;
		
    ret = true;
    }
  

  /*
  for(i = 0; i < 6; i++)
  {
      switch(i){
        case 0:
          if(kx132_register_read(kx132_XOUT_H, &buf[i], 1) == true){
            kx132_register_read(kx132_XOUT_H, &buf[i], 1);
            break;
          }
        case 1:
          if(kx132_register_read(kx132_XOUT_L, &buf[i], 1) == true){
            kx132_register_read(kx132_XOUT_L, &buf[i], 1);
            break;
          }
        case 2:
          if(kx132_register_read(kx132_YOUT_H, &buf[i], 1) == true){
            kx132_register_read(kx132_YOUT_H, &buf[i], 1);
            break;
          }
        case 3:
          if(kx132_register_read(kx132_YOUT_L, &buf[i], 1) == true){
            kx132_register_read(kx132_YOUT_L, &buf[i], 1);
            break;
          }
        case 4:
          if(kx132_register_read(kx132_ZOUT_H, &buf[i], 1) == true){
            kx132_register_read(kx132_ZOUT_H, &buf[i], 1);
            break;
          }
        case 5:
          if(kx132_register_read(kx132_ZOUT_L, &buf[i], 1) == true){
            kx132_register_read(kx132_ZOUT_L, &buf[i], 1);
            ret = true;
            break;
          }
      }
      printf("%d",buf[i]);
  }
  printf("\r\n");

  *pACC_X = (buf[0] << 8) | buf[1];
    if(*pACC_X & 0x8000) *pACC_X-=65536;

    *pACC_Y= (buf[2] << 8) | buf[3];
    if(*pACC_Y & 0x8000) *pACC_Y-=65536;

    *pACC_Z = (buf[4] << 8) | buf[5];
    if(*pACC_Z & 0x8000) *pACC_Z-=65536;

    */
  return ret;
}





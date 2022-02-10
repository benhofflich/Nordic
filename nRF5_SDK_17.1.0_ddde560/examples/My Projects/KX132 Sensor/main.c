

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "kx132.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




// main code

int main(void)
{

// initialize the logger
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Hello");
    NRF_LOG_FLUSH();
	
	
// create arrays which will hold x,y & z co-ordinates values of acc and gyro
    static int16_t AccValue[3], GyroValue[3];

    twi_master_init(); // initialize the twi 
    nrf_delay_ms(1000); // give some delay

    while(kx132_init() == false) // wait until kx132 sensor is successfully initialized
    {
      NRF_LOG_INFO("kx132 initialization failed!!!"); // if it failed to initialize then print a message
      NRF_LOG_FLUSH();
      nrf_delay_ms(1000);
    }

   NRF_LOG_INFO("kx132 Init Successfully!!!"); 
   NRF_LOG_FLUSH();

   NRF_LOG_INFO("Reading Values from ACC & GYRO"); // display a message to let the user know that the device is starting to read the values
   NRF_LOG_FLUSH();
   nrf_delay_ms(2000);


  
    
    while (true)
    {
        if(kx132_ReadAcc(&AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from kx132 internal registers and save them in the array
        {
          NRF_LOG_INFO("ACC Values:  x = %d  y = %d  z = %d", AccValue[0], AccValue[1], AccValue[2]); // display the read values
        }
        else
        {
          NRF_LOG_INFO("Reading ACC values Failed!!!"); // if reading was unsuccessful then let the user know about it
        }
        NRF_LOG_FLUSH();

       nrf_delay_ms(100); // give some delay 


    }
}

/** @} */

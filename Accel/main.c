#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ADXL355_drive.h"


int main(void)
{
    
  // Initialize the Logger module and check if any error occured during initialization
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	
    // Initialize the default backends for nrf logger
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // print the log msg over uart port
    NRF_LOG_INFO("This is log data from nordic device..");


    // variables to hold x y z values in mg
    int intValueMgX, intValueMgY, intValueMgZ;
	
    //Initialize the LEDs on board to use them
    bsp_board_init(BSP_INIT_LEDS); 
	
	
    //Call the SPI initialization function
    SPI_Init();
    nrf_delay_ms(500);
    
    // Call the ADXL355 initialization function
    ADXL355_init();

 		
    while(true)
      {
		//
		intValueMgX = ((ADXL355_read_reg(ADD_REG_XDATA3) << 12) | (ADXL355_read_reg(ADD_REG_XDATA2) << 4) | ADXL355_read_reg(ADD_REG_XDATA1));
		intValueMgY = ((ADXL355_read_reg(ADD_REG_YDATA3) << 12) | (ADXL355_read_reg(ADD_REG_YDATA2) << 4) | ADXL355_read_reg(ADD_REG_YDATA1));
                intValueMgZ = ((ADXL355_read_reg(ADD_REG_ZDATA3) << 12) | (ADXL355_read_reg(ADD_REG_ZDATA2) << 4) | ADXL355_read_reg(ADD_REG_ZDATA1));

		/* transform X value from two's complement to 16-bit int */
		intValueMgX = twoComplToInt16(intValueMgX);
		/* convert X absolute value to mg value */
		intValueMgX = intValueMgX * SENS_2G_RANGE_UG_PER_DIGIT;

		/* transform Y value from two's complement to 16-bit int */
		intValueMgY = twoComplToInt16(intValueMgY);
		/* convert Y absolute value to mg value */
		intValueMgY = intValueMgY * SENS_2G_RANGE_UG_PER_DIGIT;

		/* transform Z value from two's complement to 16-bit int */
		intValueMgZ = twoComplToInt16(intValueMgZ);
		/* convert Z absolute value to mg value */
		intValueMgZ = intValueMgZ * SENS_2G_RANGE_UG_PER_DIGIT;
                //
		NRF_LOG_INFO("X=%d Y=%d Z=%d \r\n", intValueMgX, intValueMgY, intValueMgZ);
                printf("X=%d Y=%d Z=%d \r\n", intValueMgX, intValueMgY, intValueMgZ);
		nrf_delay_ms(300);
		nrf_gpio_pin_toggle(LED_1);
      }

}

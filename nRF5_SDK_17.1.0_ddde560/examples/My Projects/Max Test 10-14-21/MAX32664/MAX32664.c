
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "MAX32664.h"
#include "nrf_delay.h"



//Initializing TWI0 instance
#define TWI_INSTANCE_ID     0

// A flag to indicate the transfer state
static volatile bool m_xfer_done = false;


// Create a Handle for the twi communication
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

uint8_t _resetPin;
uint8_t _mfioPin;
uint8_t _address;
uint32_t _writeCoefArr[3];
uint8_t _userSelectedMode;
uint8_t _sampleRate = 100;

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
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


void max32664_init(void)
{
	uint8_t _resetPin = RESET_PIN;
	uint8_t _mfioPin = 8;

    nrf_gpio_cfg_output(_mfioPin);
	nrf_gpio_cfg_output(_resetPin);
	
	//Application Mode
    nrf_gpio_pin_clear(_resetPin);
	nrf_gpio_pin_set(_mfioPin);
	nrf_delay_ms(10);
	nrf_gpio_pin_set(_resetPin);
	nrf_delay_ms(1000);


	
}
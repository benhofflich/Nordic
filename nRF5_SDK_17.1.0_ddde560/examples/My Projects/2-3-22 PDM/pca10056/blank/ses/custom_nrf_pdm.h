#ifndef __CUSTOM_NRF_PDM_H__
#define __CUSTOM_NRF_PDM_H__

#include <stdint.h>
#include <nrfx.h>
#include <hal/nrf_pdm.h>
#include <nrfx_pdm.h>

#ifdef __cplusplus
extern "C" {
#endif


/**@brief Function for starting the microphone driver.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
bool drv_mic_start(void);

/**@brief Function for stopping the microphone driver.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
bool drv_mic_stop(void);

/**@brief Function for initializing the microphone driver.
 *
 * @param[in] data_handler      Pointer data handler callback.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
uint32_t drv_mic_init(uint32_t _pin_clk, uint32_t _pin_din, uint32_t _pin_pwr);

#endif

/** @} */
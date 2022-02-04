#include <nrfx.h>

#include <nrfx_pdm.h>
#include <hal/nrf_gpio.h>

#define NRFX_LOG_MODULE PDM
#include <nrfx_log.h>
#include <custom_nrf_pdm.h>
#include <nrfx_gpiote.h>
#include <nrf_log.h>

#define EVT_TO_STR(event)                                       \
    (event == NRF_PDM_EVENT_STARTED ? "NRF_PDM_EVENT_STARTED" : \
    (event == NRF_PDM_EVENT_STOPPED ? "NRF_PDM_EVENT_STOPPED" : \
    (event == NRF_PDM_EVENT_END     ? "NRF_PDM_EVENT_END"     : \
                                      "UNKNOWN EVENT")))

typedef struct
{
    int16_t  buf[CONFIG_PDM_BUFFER_SIZE_SAMPLES];
    uint16_t samples;
    bool     free;
}pdm_buf_t;

#define PDM_BUF_NUM 6

static bool                   m_audio_enabled;          ///< Audio enabled flag.
static pdm_buf_t              m_pdm_buf[PDM_BUF_NUM];

uint32_t MIC_DOUT;
uint32_t MIC_CLK;
uint32_t SX_MIC_PWR_CTRL;

static void mic_power_on(void)
{
    nrf_gpio_cfg_input(MIC_DOUT,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_set(SX_MIC_PWR_CTRL);
}

static void mic_power_off(void)
{
    nrf_gpio_pin_clear(SX_MIC_PWR_CTRL);
    nrf_gpio_cfg_input(MIC_DOUT,NRF_GPIO_PIN_PULLDOWN);;
}

static void m_audio_buffer_handler(int16_t *p_buffer, uint16_t samples)
{
    uint32_t     err_code;
    pdm_buf_t  * p_pdm_buf = NULL;
    uint32_t     pdm_buf_addr;

    for(uint32_t i = 0; i < PDM_BUF_NUM; i++)
    {
        if ( m_pdm_buf[i].free == true )
        {
            m_pdm_buf[i].free    = false;
            m_pdm_buf[i].samples = samples;

            for (uint32_t j = 0; j < samples; j++)
            {
                m_pdm_buf[i].buf[j] = p_buffer[j];
            }

            p_pdm_buf = &m_pdm_buf[i];
            pdm_buf_addr = (uint32_t)&m_pdm_buf[i];

            break;
        }
    }


}

bool drv_mic_start(void)
{
    ret_code_t status;

    NRF_LOG_DEBUG("m_audio: Enabled\r\n");

    if(m_audio_enabled == true)
    {
        return true;
    }

    mic_power_on();

    nrf_pdm_enable();
    m_audio_enabled = nrf_pdm_enable_check();

    return m_audio_enabled;
}

bool drv_mic_stop(void)
{
    ret_code_t status;

    NRF_LOG_DEBUG("m_audio: Disabled\r\n");

    if(m_audio_enabled == false)
    {
        return true;
    }

    nrf_pdm_disable();

    m_audio_enabled = nrf_pdm_enable_check();

    mic_power_off();

    return m_audio_enabled;
}

uint32_t drv_mic_init(uint32_t _pin_clk, uint32_t _pin_din, uint32_t _pin_pwr)
{
    uint32_t err_code;

    m_audio_enabled = false;
    MIC_CLK = _pin_clk;
    MIC_DOUT = _pin_din;
    SX_MIC_PWR_CTRL = _pin_pwr;

    for(uint32_t i = 0; i < PDM_BUF_NUM; i++)
    {
        m_pdm_buf[i].free = true;
    }

    nrf_gpio_cfg_output(SX_MIC_PWR_CTRL);

    mic_power_off();

    nrfx_pdm_config_t config1 = NRFX_PDM_DEFAULT_CONFIG(MIC_CLK, MIC_DOUT);

    return nrfx_pdm_init(&config1, m_audio_buffer_handler);
}
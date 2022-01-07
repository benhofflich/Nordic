#include "nrf.h"
//#include "bsp.h"
#include "boards.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_pdm.h"
#define _pin_clk 18
#define _pin_din 16

#define FILE_NAME   "NORDIC.pcm"

#define SDC_SCK_PIN     1 //SD_CLK  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    4  //SD_DI  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    0 //SD_DO  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      5  //SD_CS  ///< SDC chip select (CS) pin.

#define LED1 NRF_GPIO_PIN_MAP(0,28)
int16_t buff1[1024];
int16_t buff2[1024];
bool flag = 0;
bool WriteFlag = 0;
int32_t interupt = 0;
int32_t pisanje = 0;

NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

static void drv_pdm_hand(const nrfx_pdm_evt_t *evt){
	//interupt++;
	//if((*evt).error != 0){int hhh = 1;} //(*evt).error, evt->error
	
	if((*evt).buffer_requested){
		if(!flag){
			//memcpy(&buff2[0], (*evt).buffer_released, 1000);
			int32_t error = nrfx_pdm_buffer_set(&buff1[0], 1024);
			flag = 1;
			WriteFlag = 1;
		}
		else{
			int32_t error = nrfx_pdm_buffer_set(&buff2[0], 1024);
			flag = 0;
			WriteFlag = 1;
		}
	}
}

int main(void)
{
	//nrf_gpio_cfg_output(LED1); 
	//bsp_board_init(BSP_INIT_LEDS);
	
	// SD card open
	static FATFS fs;
	static DIR dir;
	static FILINFO fno;
	uint32_t bytes_written;
	static FIL file;
    
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
        //return;
    }

    //uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    //uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    //NRF_LOG_INFO("Capacity: %d MB", capacity);

    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        //NRF_LOG_INFO("Mount failed.");
        //return;
    }

    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        //NRF_LOG_INFO("Directory listing failed!");
        //return;
    }

    //NRF_LOG_INFO("Writing to file " FILE_NAME "...");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        //NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
        //return;
    }
    
    // start PDM
	uint32_t error = 0;
	nrfx_pdm_config_t config1 = NRFX_PDM_DEFAULT_CONFIG(_pin_clk, _pin_din);
	error = nrfx_pdm_init(&config1, drv_pdm_hand);
	error = nrfx_pdm_start();
	
	int i = 100000; // 100.000 * 1024 samples
    
	while(i){
		while(!WriteFlag); // wait for write flag to be set
		if(flag){
			ff_result = f_write(&file,&buff1[0] , sizeof(buff1), (UINT *) &bytes_written);
			WriteFlag = 0;
		}
		else{
			ff_result = f_write(&file,&buff2[0] , sizeof(buff2), (UINT *) &bytes_written);
			WriteFlag = 0;
		}
		i--;
	}

	f_close(&file);
	
	while(1); // constant loop
}

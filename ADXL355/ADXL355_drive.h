#ifndef ADXL355_DRIVE_H
#define ADXL355_DRIVE_H


/* Pins Definition for ADXL355 */
#define SPI_SS_PIN   13  // pin to be connected to cs(chip select pin)
#define SPI_SCK_PIN  27  // pin to be connected to clock pin (scl)
#define SPI_MISO_PIN 26  // pin to be connected to MISO 
#define SPI_MOSI_PIN 2  // pin to be connected to MOSI


#define    SPIFlash_CS_LOW    nrf_gpio_pin_clear(SPI_SS_PIN)   // a simple function call to clear the cs pin
#define    SPIFlash_CS_HIGH   nrf_gpio_pin_set(SPI_SS_PIN)     //a simple function call to set the cs pin 


/* ADXL355 registers addresses */
#define ADD_REG_PARTID              0x02 //Contains device ID
#define ADD_REG_STATUS              0x04
#define ADD_REG_TEMP2               0x06
#define ADD_REG_TEMP1               0x07
#define ADD_REG_XDATA3              0x08
#define ADD_REG_XDATA2              0x09
#define ADD_REG_XDATA1              0x0A
#define ADD_REG_YDATA3              0x0B
#define ADD_REG_YDATA2              0x0C
#define ADD_REG_YDATA1              0x0D
#define ADD_REG_ZDATA3              0x0E
#define ADD_REG_ZDATA2              0x0F
#define ADD_REG_ZDATA1              0x10
#define ADD_REG_ACT_EN              0x24
#define ADD_REG_ACT_THRESH_H        0x25 //Threshold for activity detection
#define ADD_REG_ACT_THRESH_L        0x26 //Threshold for activity detection
#define ADD_REG_ACT_COUNT           0x27 //Number of consecutive events above threshold to detect activity

/* PARTID register default value */
#define UC_PARTID_DEFAULT_VALUE     0xED

/* ADD_REG_ACT_ENABL register configuration value: X,Y,Z axis enabled and 500Hz of output data rate
   for more info see the datasheet of ADXL355
 */
#define UC_ADD_REG_ACT_EN_CFG_VALUE 0x07

/* Sensitivity for 2G range [ug/digit] */
#define SENS_2G_RANGE_UG_PER_DIGIT		((float)3.9)

/* LED threshold value in mg */
#define LED_TH_MG						(1000)	/* 1000mg (1G) */


/* ---------------- Local Macros ----------------- */

/* set read single command. Attention: command must be 0x3F at most */
#define SET_READ_SINGLE_CMD(x)			(x | 0x80)
/* set read multiple command. Attention: command must be 0x3F at most */
#define SET_READ_MULTI_CMD(x)			(x | 0xC0)
/* set write single command. Attention: command must be 0x3F at most */
#define SET_WRITE_SINGLE_CMD(x)			(x & (~(0xC0)))
/* set write multiple command. Attention: command must be 0x3F at most */
#define SET_WRITE_MULTI_CMD(x)			(x & (~(0x80))	\
                                                 x |= 0x40)


void SPI_Init(void);                      /* A function to intialize the SPI communication */
void ADXL355_init(void);                  /* Funtiont to initialize ADXL355 sensor */
int twoComplToInt16(int twoComplValue);   /* A function to convert 2's complement values to int16 values */
int ADXL355_read_reg(int reg);            /* A function to read a value from a register */

#endif
/********************************************END FILE*******************************************/


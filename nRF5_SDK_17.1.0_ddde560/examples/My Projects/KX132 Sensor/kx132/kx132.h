#ifndef KX132_H
#define KX132_H
#include "nrf_delay.h"

//I2C Pins Settings, you change them to any other pins
#define TWI_SCL_M           15         //I2C SCL Pin
#define TWI_SDA_M           14         //I2C SDA Pin


#define kx132_ADDRESS_LEN  1         //kx132
#define kx132_ADDRESS      0x1F      //kx132 Device Address
#define kx132_WHO_AM_I     0x3D     //kx132 ID


#define kx132_ACC_OUT         0x09

//kx132 Registers addresses, see datasheet for more info and each register's function
enum kx132_REGISTERS {

  kx132_MAN_ID = 0x00,//      Retuns "KION" in ASCII
  kx132_PART_ID,//            Who am I + Silicon specific ID
  kx132_XADP_L,//     ------- X, Y, and Z - High and Low bytes -----
  kx132_XADP_H,
  kx132_YADP_L,
  kx132_YADP_H,
  kx132_ZADP_L,
  kx132_ZADP_H,
  kx132_XOUT_L,
  kx132_XOUT_H,
  kx132_YOUT_L,
  kx132_YOUT_H,
  kx132_ZOUT_L,
  kx132_ZOUT_H, //     --------------^^--------------------------
  //                          0x0E - 0x11 Reserved
  kx132_COTR = 0x12,//        Command Test Register
  WHO_AM_I, //          Who am I: 0x3D-KX132, 0x46-KX134
  KXI3X_TSCP,//        -------Tilt Register---------------------- 
  kx132_TSPP,//        -----------^^-----------------------------
  kx132_INS1, //        -------Interrupt Registers ---------------
  kx132_INS2,
  kx132_INS3,
  kx132_STATUS_REG, 
  kx132_INT_REL, //    ------------^^----------------------------
  kx132_CNTL1,//       --------Control Registers----------------- 
  kx132_CNTL2,
  kx132_CNTL3,
  kx132_CNTL4,
  kx132_CNTL5,
  kx132_CNTL6,//        -------------^^---------------------------
  kx132_ODCNTL,
  kx132_INC1,//Controls settings for INT1
  kx132_INC2,//Defines behavior for Wake-Up Function and Back To Sleep
  kx132_INC3,//Defines which axes can cause a tap based interrupt
  kx132_INC4,//Controls which function triggers INT1
  kx132_INC5,
  kx132_INC6,//Controls which function triggers INT2
  // 0x28 Reserved
  kx132_TILT_TIMER =  0x29, 
  kx132_TDTRC, // Tap Control Regs ----- 
  kx132_TDTC,
  kx132_TTH,
  kx132_TTL,
  kx132_FTD,
  kx132_STD,
  kx132_TLT,
  kx132_TWS,
  kx132_FFTH,
  kx132_FFC,
  kx132_FFCNTL,
  // 0x35 - 0x36 Reserved
  kx132_TILT_ANGLE_LL = 0x37,
  kx132_TILT_ANGLE_HL,
  kx132_HYST_SET,
  kx132_LP_CNTL1,
  kx132_LP_CNTL2,
  // 0x3C - 0x48 Reserved
  kx132_WUFTH = 0x49,
  kx132_BTSWUFTH,
  kx132_BTSTH,
  kx132_BTSC,
  kx132_WUFC,
  // 0x4E - 0x5C Reserved
  kx132_SELF_TEST = 0x5D,
  kx132_BUF_CNTL1,
  kx132_BUF_CNTL2,
  kx132_BUF_STATUS_1,
  kx132_BUF_STATUS_2,
  kx132_BUF_CLEAR,
  kx132_BUF_READ,
  kx132_ADP_CNTL1,
  kx132_ADP_CNTL2,
  kx132_ADP_CNTL3,
  kx132_ADP_CNTL4,
  kx132_ADP_CNTL5,
  kx132_ADP_CNTL6,
  kx132_ADP_CNTL7,
  kx132_ADP_CNTL8,
  kx132_ADP_CNTL9,
  kx132_ADP_CNTL10,
  kx132_ADP_CNTL11,
  kx132_ADP_CNTL12,
  kx132_ADP_CNTL13,
  kx132_ADP_CNTL14,
  kx132_ADP_CNTL15,
  kx132_ADP_CNTL16,
  kx132_ADP_CNTL17,
  kx132_ADP_CNTL18,
  kx132_ADP_CNTL19
  //Reserved 0x77 - 0x7F
};


void twi_master_init(void); // initialize the twi communication
bool kx132_init(void);    // initialize the kx132

/**
  @brief Function for writing a kx132 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool kx132_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading kx132 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool kx132_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying kx132 product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool kx132_verify_product_id(void);


bool kx132_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z );

#endif



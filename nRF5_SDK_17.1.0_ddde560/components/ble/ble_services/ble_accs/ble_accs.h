/**
 * Copyright (c) 2012 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_accs ACC Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief ACC Service module.
 *
 * @details This module implements the ACC Service with the ACC Measurement characteristics.
 *          During initialization it adds the ACC Service and ACC Measurement
 *          characteristic to the BLE stack database. 
 *
 *          If enabled, notification of the ACC Measurement characteristic is performed
 *          when the application calls ble_accs_measurement_send().
 *
 *          The ACC Service also provides a set of functions for manipulating the
 *          various fields in the ACC Measurement characteristic.
 *          If an event handler is supplied by the application, the ACC Service will
 *          generate ACC Service events to the application.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_accs_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_ACCS_BLE_OBSERVER_PRIO,
 *                                   ble_accs_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_ACCS_H__
#define BLE_ACCS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif


/**@brief   Macro for defining a ble_accs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ACCS_DEF(_name)                                                                          \
static ble_accs_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_ACCS_BLE_OBSERVER_PRIO,                                                     \
                     ble_accs_on_ble_evt, &_name)

// ACC Service UUID: 67963E96-A292-4341-9A0F-83DDD852B8D5
// Accelerometer Characteristic UUID: 67963E97-A292-4341-9A0F-83DDD852B8D5

#define BLE_UUID_ACC_SERVICE_BASE_UUID {0xD5, 0xB8, 0x52, 0xD8, 0xDD, 0x83, 0x0F, 0x9A, 0x41, 0x43, 0x92, 0xA2, 0x96, 0x3E, 0x96, 0x67}
#define BLE_UUID_ACC_SERVICE_UUID       0x3E96

#define BLE_UUID_ACCELEROMETER_X_CHAR_BASE_UUID {0xD5, 0xB8, 0x52, 0xD8, 0xDD, 0x83, 0x0F, 0x9A, 0x41, 0x43, 0x92, 0xA2, 0x97, 0x3E, 0x96, 0x67}
#define BLE_UUID_ACCELEROMETER_X_CHAR_UUID       0x3E97

#define BLE_UUID_ACCELEROMETER_Y_CHAR_BASE_UUID {0xD5, 0xB8, 0x52, 0xD8, 0xDD, 0x83, 0x0F, 0x9A, 0x41, 0x43, 0x92, 0xA2, 0x98, 0x3E, 0x96, 0x67}
#define BLE_UUID_ACCELEROMETER_Y_CHAR_UUID       0x3E98

#define BLE_UUID_ACCELEROMETER_Z_CHAR_BASE_UUID {0xD5, 0xB8, 0x52, 0xD8, 0xDD, 0x83, 0x0F, 0x9A, 0x41, 0x43, 0x92, 0xA2, 0x99, 0x3E, 0x96, 0x67}
#define BLE_UUID_ACCELEROMETER_Z_CHAR_UUID       0x3E99

/**@brief ACC Service event type. */
typedef enum
{
    BLE_ACCS_EVT_NOTIFICATION_ENABLED,   /**< ACC value notification enabled event. */
    BLE_ACCS_EVT_NOTIFICATION_DISABLED   /**< ACC value notification disabled event. */
} ble_accs_evt_type_t;

/**@brief ACC Service event. */
typedef struct
{
    ble_accs_evt_type_t evt_type;    /**< Type of event. */
} ble_accs_evt_t;

// Forward declaration of the ble_accs_t type.
typedef struct ble_accs_s ble_accs_t;

/**@brief ACC Service event handler type. */
typedef void (*ble_accs_evt_handler_t) (ble_accs_t * p_accs, ble_accs_evt_t * p_evt);

/**@brief ACC Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_accs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the ACC Service. */
    security_req_t               accm_cccd_wr_sec;                                      /**< Security requirement for writing the ACC characteristic CCCD. */
    security_req_t               bsl_rd_sec;                                           /**< Security requirement for reading the BSL characteristic value. */
} ble_accs_init_t;

/**@brief ACC Service structure. This contains various status information for the service. */
struct ble_accs_s
{
    ble_accs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the ACC Service. */
    bool                         is_expended_energy_supported;                         /**< TRUE if Expended Energy measurement is supported. */
    uint16_t                     service_handle;                                       /**< Handle of ACC Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     accxm_handles;                                          /**< Handles related to the ACCx Measurement characteristic. */
    ble_gatts_char_handles_t     accym_handles;                                          /**< Handles related to the ACCy Measurement characteristic. */
    ble_gatts_char_handles_t     acczm_handles;                                          /**< Handles related to the ACCz Measurement characteristic. */
    ble_gatts_char_handles_t     acccp_handles;                                         /**< Handles related to the ACC Control Point characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                      max_accm_len;                                          /**< Current maximum ACC measurement length, adjusted according to the current ATT MTU. */
    uint8_t                      uuid_type;
};


/**@brief Function for initializing the ACC Service.
 *
 * @param[out]  p_accs       ACC Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_accs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_accs_init(ble_accs_t * p_accs, ble_accs_init_t const * p_accs_init);


/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the ACC Service.
 *
 * @param[in]   p_accs      ACC Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_accs_on_gatt_evt(ble_accs_t * p_accs, nrf_ble_gatt_evt_t const * p_gatt_evt);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the ACC Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   ACC Service structure.
 */
void ble_accs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending ACC measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a ACC measurement.
 *          If notification has been enabled, the ACC measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_accs                    ACC Service structure.
 * @param[in]   acc                       New ACC measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_accs_acc_measurement_send(ble_accs_t * p_accs, uint16_t xAcc, uint16_t yAcc, uint16_t zAcc);


#ifdef __cplusplus
}
#endif

#endif // BLE_ACCS_H__

/** @} */

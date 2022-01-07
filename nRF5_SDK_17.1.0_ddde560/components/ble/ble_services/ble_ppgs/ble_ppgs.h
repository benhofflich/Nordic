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
 * @defgroup ble_ppgs PPG Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief PPG Service module.
 *
 * @details This module implements the PPG Service with the PPG Measurement characteristics.
 *          During initialization it adds the PPG Service and PPG Measurement
 *          characteristic to the BLE stack database. 
 *
 *          If enabled, notification of the PPG Measurement characteristic is performed
 *          when the application calls ble_ppgs_oxygen_measurement_send().
 *
 *          The PPG Service also provides a set of functions for manipulating the
 *          various fields in the PPG Measurement characteristic.
 *          If an event handler is supplied by the application, the PPG Service will
 *          generate PPG Service events to the application.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_ppgs_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_PPGS_BLE_OBSERVER_PRIO,
 *                                   ble_ppgs_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_PPGS_H__
#define BLE_PPGS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

// Body Sensor Location values
#define BLE_PPGS_BODY_SENSOR_LOCATION_NONE       0
#define BLE_PPGS_BODY_SENSOR_LOCATION_SOMETHING  1
#define BLE_PPGS_BODY_SENSOR_LOCATION_OTHER      2
#define BLE_PPGS_BODY_SENSOR_LOCATION_FINGER     3

#define BLE_PPGS_MAX_BUFFERED_RR_INTERVALS       20      /**< Size of RR Interval buffer inside service. */

/**@brief   Macro for defining a ble_ppgs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_PPGS_DEF(_name)                                                                          \
static ble_ppgs_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_PPGS_BLE_OBSERVER_PRIO,                                                     \
                     ble_ppgs_on_ble_evt, &_name)

// PPG Service UUID: BE5E5D21-959C-4B50-A8EB-A43A7F6E29D1
// Red Counter Characteristic UUID: BE5E5D22-959C-4B50-A8EB-A43A7F6E29D1
// Infrared Characteristic UUID: BE5E5D23-959C-4B50-A8EB-A43A7F6E29D1

#define BLE_UUID_PPG_SERVICE_BASE_UUID {0xD1, 0x29, 0x6E, 0x7F, 0x3A, 0xA4, 0xEB, 0xA8, 0x50, 0x4B, 0x9C, 0x95, 0x21, 0x5D, 0x5E, 0xBE}
#define BLE_UUID_PPG_SERVICE_UUID       0x5D21

#define BLE_UUID_RED_COUNTER_CHAR_BASE_UUID {0xD1, 0x29, 0x6E, 0x7F, 0x3A, 0xA4, 0xEB, 0xA8, 0x50, 0x4B, 0x9C, 0x95, 0x22, 0x5D, 0x5E, 0xBE}
#define BLE_UUID_RED_COUNTER_CHAR_UUID       0x5D22

#define BLE_UUID_INFRARED_CHAR_BASE_UUID {0xD1, 0x29, 0x6E, 0x7F, 0x3A, 0xA4, 0xEB, 0xA8, 0x50, 0x4B, 0x9C, 0x95, 0x23, 0x5D, 0x5E, 0xBE}
#define BLE_UUID_INFRARED_CHAR_UUID       0x5D23

/**@brief PPG Service event type. */
typedef enum
{
    BLE_PPGS_EVT_NOTIFICATION_ENABLED,   /**< PPG value notification enabled event. */
    BLE_PPGS_EVT_NOTIFICATION_DISABLED   /**< PPG value notification disabled event. */
} ble_ppgs_evt_type_t;

/**@brief PPG Service event. */
typedef struct
{
    ble_ppgs_evt_type_t evt_type;    /**< Type of event. */
} ble_ppgs_evt_t;

// Forward declaration of the ble_ppgs_t type.
typedef struct ble_ppgs_s ble_ppgs_t;

/**@brief PPG Service event handler type. */
typedef void (*ble_ppgs_evt_handler_t) (ble_ppgs_t * p_ppgs, ble_ppgs_evt_t * p_evt);

/**@brief PPG Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_ppgs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the PPG Service. */
    bool                         is_sensor_contact_supported;                          /**< Determines if sensor contact detection is to be supported. */
    uint8_t *                    p_body_sensor_location;                               /**< If not NULL, initial value of the Body Sensor Location characteristic. */
    security_req_t               ppgm_cccd_wr_sec;                                      /**< Security requirement for writing the SPO2 characteristic CCCD. */
    security_req_t               bsl_rd_sec;                                           /**< Security requirement for reading the BSL characteristic value. */
} ble_ppgs_init_t;

/**@brief PPG Service structure. This contains various status information for the service. */
struct ble_ppgs_s
{
    ble_ppgs_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the PPG Service. */
    bool                         is_expended_energy_supported;                         /**< TRUE if Expended Energy measurement is supported. */
    bool                         is_sensor_contact_supported;                          /**< TRUE if sensor contact detection is supported. */
    uint16_t                     service_handle;                                       /**< Handle of PPG Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     rcm_handles;                                          /**< Handles related to the PPG Measurement characteristic. */
    ble_gatts_char_handles_t     irm_handles;                                          /**< Handles related to the PPG Measurement characteristic. */
    ble_gatts_char_handles_t     bsl_handles;                                          /**< Handles related to the Body Sensor Location characteristic. */
    ble_gatts_char_handles_t     ppgcp_handles;                                         /**< Handles related to the PPG Control Point characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                         is_sensor_contact_detected;                           /**< TRUE if sensor contact has been detected. */
    uint16_t                     rr_interval[BLE_PPGS_MAX_BUFFERED_RR_INTERVALS];       /**< Set of RR Interval measurements since the last PPG Measurement transmission. */
    uint16_t                     rr_interval_count;                                    /**< Number of RR Interval measurements since the last PPG Measurement transmission. */
    uint8_t                      max_ppgm_len;                                          /**< Current maximum PPG measurement length, adjusted according to the current ATT MTU. */
    uint8_t                      uuid_type;
};


/**@brief Function for initializing the PPG Service.
 *
 * @param[out]  p_ppgs       PPG Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_ppgs_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ppgs_init(ble_ppgs_t * p_ppgs, ble_ppgs_init_t const * p_ppgs_init);


/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the PPG Service.
 *
 * @param[in]   p_ppgs      PPG Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_ppgs_on_gatt_evt(ble_ppgs_t * p_ppgs, nrf_ble_gatt_evt_t const * p_gatt_evt);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the PPG Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   PPG Service structure.
 */
void ble_ppgs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending PPG measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a PPG measurement.
 *          If notification has been enabled, the PPG measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_ppgs                    PPG Service structure.
 * @param[in]   ppg                       New PPG measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ppgs_ir_measurement_send(ble_ppgs_t * p_ppgs, uint32_t infrared);
uint32_t ble_ppgs_rc_measurement_send(ble_ppgs_t * p_ppgs, uint32_t redCounter);


/**@brief Function for adding a RR Interval measurement to the RR Interval buffer.
 *
 * @details All buffered RR Interval measurements will be included in the next PPG
 *          measurement message, up to the maximum number of measurements that will fit into the
 *          message. If the buffer is full, the oldest measurement in the buffer will be deleted.
 *
 * @param[in]   p_ppgs        PPG Service structure.
 * @param[in]   rr_interval  New RR Interval measurement (will be buffered until the next
 *                           transmission of PPG Measurement).
 */
void ble_ppgs_rr_interval_add(ble_ppgs_t * p_ppgs, uint16_t rr_interval);


/**@brief Function for checking if RR Interval buffer is full.
 *
 * @param[in]   p_ppgs        PPG Service structure.
 *
 * @return      true if RR Interval buffer is full, false otherwise.
 */
bool ble_ppgs_rr_interval_buffer_is_full(ble_ppgs_t * p_ppgs);


/**@brief Function for setting the state of the Sensor Contact Supported bit.
 *
 * @param[in]   p_ppgs                        PPG Service structure.
 * @param[in]   is_sensor_contact_supported  New state of the Sensor Contact Supported bit.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ppgs_sensor_contact_supported_set(ble_ppgs_t * p_ppgs, bool is_sensor_contact_supported);


/**@brief Function for setting the state of the Sensor Contact Detected bit.
 *
 * @param[in]   p_ppgs                        PPG Service structure.
 * @param[in]   is_sensor_contact_detected   TRUE if sensor contact is detected, FALSE otherwise.
 */
void ble_ppgs_sensor_contact_detected_update(ble_ppgs_t * p_ppgs, bool is_sensor_contact_detected);


/**@brief Function for setting the Body Sensor Location.
 *
 * @details Sets a new value of the Body Sensor Location characteristic. The new value will be sent
 *          to the client the next time the client reads the Body Sensor Location characteristic.
 *
 * @param[in]   p_ppgs                 PPG Service structure.
 * @param[in]   body_sensor_location  New Body Sensor Location.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ppgs_body_sensor_location_set(ble_ppgs_t * p_ppgs, uint8_t body_sensor_location);


#ifdef __cplusplus
}
#endif

#endif // BLE_PPGS_H__

/** @} */

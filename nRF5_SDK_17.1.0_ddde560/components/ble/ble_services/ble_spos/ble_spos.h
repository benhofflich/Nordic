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
 * @defgroup ble_spos SpO2 Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief SpO2 Service module.
 *
 * @details This module implements the SpO2 Service with the SpO2 Measurement characteristics.
 *          During initialization it adds the SpO2 Service and SpO2 Measurement
 *          characteristic to the BLE stack database. 
 *
 *          If enabled, notification of the SpO2 Measurement characteristic is performed
 *          when the application calls ble_spos_oxygen_measurement_send().
 *
 *          The SpO2 Service also provides a set of functions for manipulating the
 *          various fields in the SpO2 Measurement characteristic.
 *          If an event handler is supplied by the application, the SpO2 Service will
 *          generate SpO2 Service events to the application.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_spos_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_SPOS_BLE_OBSERVER_PRIO,
 *                                   ble_spos_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_SPOS_H__
#define BLE_SPOS_H__

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
#define BLE_SPOS_BODY_SENSOR_LOCATION_NONE       0
#define BLE_SPOS_BODY_SENSOR_LOCATION_SOMETHING  1
#define BLE_SPOS_BODY_SENSOR_LOCATION_OTHER      2
#define BLE_SPOS_BODY_SENSOR_LOCATION_FINGER     3

#define BLE_SPOS_MAX_BUFFERED_RR_INTERVALS       20      /**< Size of RR Interval buffer inside service. */

/**@brief   Macro for defining a ble_spos instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_SPOS_DEF(_name)                                                                          \
static ble_spos_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_SPOS_BLE_OBSERVER_PRIO,                                                     \
                     ble_spos_on_ble_evt, &_name)


/**@brief SpO2 Service event type. */
typedef enum
{
    BLE_SPOS_EVT_NOTIFICATION_ENABLED,   /**< SpO2 value notification enabled event. */
    BLE_SPOS_EVT_NOTIFICATION_DISABLED   /**< SpO2 value notification disabled event. */
} ble_spos_evt_type_t;

/**@brief SpO2 Service event. */
typedef struct
{
    ble_spos_evt_type_t evt_type;    /**< Type of event. */
} ble_spos_evt_t;

// Forward declaration of the ble_spos_t type.
typedef struct ble_spos_s ble_spos_t;

/**@brief SpO2 Service event handler type. */
typedef void (*ble_spos_evt_handler_t) (ble_spos_t * p_spos, ble_spos_evt_t * p_evt);

/**@brief SpO2 Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_spos_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the SpO2 Service. */
    bool                         is_sensor_contact_supported;                          /**< Determines if sensor contact detection is to be supported. */
    uint8_t *                    p_body_sensor_location;                               /**< If not NULL, initial value of the Body Sensor Location characteristic. */
    security_req_t               spom_cccd_wr_sec;                                      /**< Security requirement for writing the SPO2 characteristic CCCD. */
    security_req_t               bsl_rd_sec;                                           /**< Security requirement for reading the BSL characteristic value. */
} ble_spos_init_t;

/**@brief SpO2 Service structure. This contains various status information for the service. */
struct ble_spos_s
{
    ble_spos_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the SpO2 Service. */
    bool                         is_expended_energy_supported;                         /**< TRUE if Expended Energy measurement is supported. */
    bool                         is_sensor_contact_supported;                          /**< TRUE if sensor contact detection is supported. */
    uint16_t                     service_handle;                                       /**< Handle of SpO2 Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     spom_handles;                                          /**< Handles related to the SpO2 Measurement characteristic. */
    ble_gatts_char_handles_t     bsl_handles;                                          /**< Handles related to the Body Sensor Location characteristic. */
    ble_gatts_char_handles_t     spocp_handles;                                         /**< Handles related to the SpO2 Control Point characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                         is_sensor_contact_detected;                           /**< TRUE if sensor contact has been detected. */
    uint16_t                     rr_interval[BLE_SPOS_MAX_BUFFERED_RR_INTERVALS];       /**< Set of RR Interval measurements since the last SpO2 Measurement transmission. */
    uint16_t                     rr_interval_count;                                    /**< Number of RR Interval measurements since the last SpO2 Measurement transmission. */
    uint8_t                      max_spom_len;                                          /**< Current maximum HR measurement length, adjusted according to the current ATT MTU. */
};


/**@brief Function for initializing the SpO2 Service.
 *
 * @param[out]  p_spos       SpO2 Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_spos_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_spos_init(ble_spos_t * p_spos, ble_spos_init_t const * p_spos_init);


/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the SpO2 Service.
 *
 * @param[in]   p_spos      SpO2 Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_spos_on_gatt_evt(ble_spos_t * p_spos, nrf_ble_gatt_evt_t const * p_gatt_evt);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the SpO2 Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   SpO2 Service structure.
 */
void ble_spos_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending SpO2 measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a SpO2 measurement.
 *          If notification has been enabled, the SpO2 measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_spos                    SpO2 Service structure.
 * @param[in]   heart_rate               New SpO2 measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_spos_spo2_measurement_send(ble_spos_t * p_spos, uint16_t heart_rate);


/**@brief Function for adding a RR Interval measurement to the RR Interval buffer.
 *
 * @details All buffered RR Interval measurements will be included in the next SpO2
 *          measurement message, up to the maximum number of measurements that will fit into the
 *          message. If the buffer is full, the oldest measurement in the buffer will be deleted.
 *
 * @param[in]   p_spos        SpO2 Service structure.
 * @param[in]   rr_interval  New RR Interval measurement (will be buffered until the next
 *                           transmission of SpO2 Measurement).
 */
void ble_spos_rr_interval_add(ble_spos_t * p_spos, uint16_t rr_interval);


/**@brief Function for checking if RR Interval buffer is full.
 *
 * @param[in]   p_spos        SpO2 Service structure.
 *
 * @return      true if RR Interval buffer is full, false otherwise.
 */
bool ble_spos_rr_interval_buffer_is_full(ble_spos_t * p_spos);


/**@brief Function for setting the state of the Sensor Contact Supported bit.
 *
 * @param[in]   p_spos                        SpO2 Service structure.
 * @param[in]   is_sensor_contact_supported  New state of the Sensor Contact Supported bit.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_spos_sensor_contact_supported_set(ble_spos_t * p_spos, bool is_sensor_contact_supported);


/**@brief Function for setting the state of the Sensor Contact Detected bit.
 *
 * @param[in]   p_spos                        SpO2 Service structure.
 * @param[in]   is_sensor_contact_detected   TRUE if sensor contact is detected, FALSE otherwise.
 */
void ble_spos_sensor_contact_detected_update(ble_spos_t * p_spos, bool is_sensor_contact_detected);


/**@brief Function for setting the Body Sensor Location.
 *
 * @details Sets a new value of the Body Sensor Location characteristic. The new value will be sent
 *          to the client the next time the client reads the Body Sensor Location characteristic.
 *
 * @param[in]   p_spos                 SpO2 Service structure.
 * @param[in]   body_sensor_location  New Body Sensor Location.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_spos_body_sensor_location_set(ble_spos_t * p_spos, uint8_t body_sensor_location);


#ifdef __cplusplus
}
#endif

#endif // BLE_SPOS_H__

/** @} */

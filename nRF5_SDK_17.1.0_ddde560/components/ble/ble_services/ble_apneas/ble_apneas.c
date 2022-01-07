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
/* Attention!
 * To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 * qualification listings, this section of source code must not be modified.
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_HRS)
#include "ble_apneas.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside Heart Rate Measurement packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside Heart Rate Measurement packet. */
#define MAX_HRM_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted Heart Rate Measurement. */

#define INITIAL_VALUE_HRM                       0                                    /**< Initial Heart Rate Measurement value. */

// Heart Rate Measurement flag bits
#define HRM_FLAG_MASK_HR_VALUE_16BIT            (0x01 << 0)                           /**< Heart Rate Value Format bit. */
#define HRM_FLAG_MASK_PPG_VALUE_32BIT           (0x01 << 1)                           /**< PPG Value Format bit. */
#define HRM_FLAG_MASK_SENSOR_CONTACT_DETECTED   (0x01 << 1)                           /**< Sensor Contact Detected bit. */
#define HRM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED  (0x01 << 2)                           /**< Sensor Contact Supported bit. */
#define HRM_FLAG_MASK_EXPENDED_ENERGY_INCLUDED  (0x01 << 3)                           /**< Energy Expended Status bit. Feature Not Supported */
#define HRM_FLAG_MASK_RR_INTERVAL_INCLUDED      (0x01 << 4)                           /**< RR-Interval bit. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_apneas       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_apneas_t * p_apneas, ble_evt_t const * p_ble_evt)
{
    p_apneas->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_apneas       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_apneas_t * p_apneas, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_apneas->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_apneas         Heart Rate Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_hrm_cccd_write(ble_apneas_t * p_apneas, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_apneas->evt_handler != NULL)
        {
            ble_apneas_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_APNEAS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_APNEAS_EVT_NOTIFICATION_DISABLED;
            }

            p_apneas->evt_handler(p_apneas, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_apneas       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_apneas_t * p_apneas, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_apneas->hrm_handles.cccd_handle)
    {
        on_hrm_cccd_write(p_apneas, p_evt_write);
    }
}


void ble_apneas_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_apneas_t * p_apneas = (ble_apneas_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_apneas, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_apneas, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_apneas, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a Heart Rate Measurement.
 *
 * @param[in]   p_apneas              Heart Rate Service structure.
 * @param[in]   heart_rate         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t hrm_encode(ble_apneas_t * p_apneas, uint16_t heart_rate, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Set sensor contact related flags
    if (p_apneas->is_sensor_contact_supported)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    }
    if (p_apneas->is_sensor_contact_detected)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    }

    // Encode heart rate measurement
    if (heart_rate > 0xff)
    {
        flags |= HRM_FLAG_MASK_HR_VALUE_16BIT;
        len   += uint16_encode(heart_rate, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)heart_rate;
    }

    // Encode rr_interval values
    if (p_apneas->rr_interval_count > 0)
    {
        flags |= HRM_FLAG_MASK_RR_INTERVAL_INCLUDED;
    }
    for (i = 0; i < p_apneas->rr_interval_count; i++)
    {
        if (len + sizeof(uint16_t) > p_apneas->max_hrm_len)
        {
            // Not all stored rr_interval values can fit into the encoded hrm,
            // move the remaining values to the start of the buffer.
            memmove(&p_apneas->rr_interval[0],
                    &p_apneas->rr_interval[i],
                    (p_apneas->rr_interval_count - i) * sizeof(uint16_t));
            break;
        }
        len += uint16_encode(p_apneas->rr_interval[i], &p_encoded_buffer[len]);
    }
    p_apneas->rr_interval_count -= i;

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}

static uint8_t ppgm_encode(ble_apneas_t * p_apneas, uint32_t ppg, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Set sensor contact related flags
    if (p_apneas->is_sensor_contact_supported)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    }
    if (p_apneas->is_sensor_contact_detected)
    {
        flags |= HRM_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    }

    // Encode heart rate measurement
    if (ppg > 0xffff)
    {
        flags |= HRM_FLAG_MASK_PPG_VALUE_32BIT;
        len   += uint32_encode(ppg, &p_encoded_buffer[len]);
    }
    else if (ppg > 0xff)
    {
        flags |= HRM_FLAG_MASK_HR_VALUE_16BIT;
        len   += uint16_encode((uint16_t)ppg, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)ppg;
    }

    // Encode rr_interval values
    if (p_apneas->rr_interval_count > 0)
    {
        flags |= HRM_FLAG_MASK_RR_INTERVAL_INCLUDED;
    }
    for (i = 0; i < p_apneas->rr_interval_count; i++)
    {
        if (len + sizeof(uint16_t) > p_apneas->max_hrm_len)
        {
            // Not all stored rr_interval values can fit into the encoded hrm,
            // move the remaining values to the start of the buffer.
            memmove(&p_apneas->rr_interval[0],
                    &p_apneas->rr_interval[i],
                    (p_apneas->rr_interval_count - i) * sizeof(uint16_t));
            break;
        }
        len += uint16_encode(p_apneas->rr_interval[i], &p_encoded_buffer[len]);
    }
    p_apneas->rr_interval_count -= i;

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}

uint32_t ble_apneas1_init(ble_apneas_t * p_apneas, const ble_apneas_init_t * p_apneas_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_hrm[MAX_HRM_LEN];

    // Initialize service structure
    p_apneas->evt_handler                 = p_apneas_init->evt_handler;
    p_apneas->is_sensor_contact_supported = p_apneas_init->is_sensor_contact_supported;
    p_apneas->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_apneas->is_sensor_contact_detected  = false;
    p_apneas->rr_interval_count           = 0;
    p_apneas->max_hrm_len                 = MAX_HRM_LEN;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEART_RATE_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_apneas->service_handle1);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add heart rate measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_HEART_RATE_MEASUREMENT_CHAR;
    add_char_params.max_len           = MAX_HRM_LEN;
    add_char_params.init_len          = hrm_encode(p_apneas, INITIAL_VALUE_HRM, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_apneas_init->hrm_cccd_wr_sec;

    err_code = characteristic_add(p_apneas->service_handle1, &add_char_params, &(p_apneas->hrm_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add SpO2 measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_PLX_CONTINUOUS_MEAS;
    add_char_params.max_len           = MAX_HRM_LEN;
    add_char_params.init_len          = hrm_encode(p_apneas, INITIAL_VALUE_HRM, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_apneas_init->hrm_cccd_wr_sec;

    err_code = characteristic_add(p_apneas->service_handle1, &add_char_params, &(p_apneas->spo2m_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_apneas2_init(ble_apneas_t * p_apneas, const ble_apneas_init_t * p_apneas_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_hrm[MAX_HRM_LEN];

    // Initialize service structure
    p_apneas->evt_handler                 = p_apneas_init->evt_handler;
    p_apneas->is_sensor_contact_supported = p_apneas_init->is_sensor_contact_supported;
    p_apneas->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_apneas->is_sensor_contact_detected  = false;
    p_apneas->rr_interval_count           = 0;
    p_apneas->max_hrm_len                 = MAX_HRM_LEN;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_PLX_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_apneas->service_handle2);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add RC measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_PLX_SPOT_CHECK_MEAS;
    add_char_params.max_len           = MAX_HRM_LEN;
    add_char_params.init_len          = ppgm_encode(p_apneas, INITIAL_VALUE_HRM, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_apneas_init->hrm_cccd_wr_sec;

    err_code = characteristic_add(p_apneas->service_handle2, &add_char_params, &(p_apneas->rcm_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add IR measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_PLX_FEATURES;
    add_char_params.max_len           = MAX_HRM_LEN;
    add_char_params.init_len          = ppgm_encode(p_apneas, INITIAL_VALUE_HRM, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_apneas_init->hrm_cccd_wr_sec;

    err_code = characteristic_add(p_apneas->service_handle2, &add_char_params, &(p_apneas->irm_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_apneas3_init(ble_apneas_t * p_apneas, const ble_apneas_init_t * p_apneas_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_hrm[MAX_HRM_LEN];

    // Initialize service structure
    p_apneas->evt_handler                 = p_apneas_init->evt_handler;
    p_apneas->is_sensor_contact_supported = p_apneas_init->is_sensor_contact_supported;
    p_apneas->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_apneas->is_sensor_contact_detected  = false;
    p_apneas->rr_interval_count           = 0;
    p_apneas->max_hrm_len                 = MAX_HRM_LEN;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_GLUCOSE_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_apneas->service_handle3);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Accelx measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_GLUCOSE_FEATURE_CHAR;
    add_char_params.max_len           = MAX_HRM_LEN;
    add_char_params.init_len          = hrm_encode(p_apneas, INITIAL_VALUE_HRM, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_apneas_init->hrm_cccd_wr_sec;

    err_code = characteristic_add(p_apneas->service_handle3, &add_char_params, &(p_apneas->accelxm_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Accely measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_GLUCOSE_MEASUREMENT_CHAR;
    add_char_params.max_len           = MAX_HRM_LEN;
    add_char_params.init_len          = hrm_encode(p_apneas, INITIAL_VALUE_HRM, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_apneas_init->hrm_cccd_wr_sec;

    err_code = characteristic_add(p_apneas->service_handle3, &add_char_params, &(p_apneas->accelym_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Accelz measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_GLUCOSE_MEASUREMENT_CONTEXT_CHAR;
    add_char_params.max_len           = MAX_HRM_LEN;
    add_char_params.init_len          = hrm_encode(p_apneas, INITIAL_VALUE_HRM, encoded_initial_hrm);
    add_char_params.p_init_value      = encoded_initial_hrm;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_apneas_init->hrm_cccd_wr_sec;

    err_code = characteristic_add(p_apneas->service_handle3, &add_char_params, &(p_apneas->accelzm_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_apneas_heart_rate_measurement_send(ble_apneas_t * p_apneas, uint16_t heart_rate)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_apneas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hrm[MAX_HRM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = hrm_encode(p_apneas, heart_rate, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_apneas->hrm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_apneas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_apneas_oxygen_measurement_send(ble_apneas_t * p_apneas, uint16_t oxygen)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_apneas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hrm[MAX_HRM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = hrm_encode(p_apneas, oxygen, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_apneas->spo2m_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_apneas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_apneas_red_counter_measurement_send(ble_apneas_t * p_apneas, uint32_t rc)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_apneas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hrm[MAX_HRM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = ppgm_encode(p_apneas, rc, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_apneas->rcm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_apneas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_apneas_infrared_measurement_send(ble_apneas_t * p_apneas, uint32_t ir)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_apneas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hrm[MAX_HRM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = ppgm_encode(p_apneas, ir, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_apneas->irm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_apneas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_apneas_accel_measurement_send(ble_apneas_t * p_apneas, uint16_t accelx, uint16_t accely, uint16_t accelz)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_apneas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_hrm[MAX_HRM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        //x char
        len     = hrm_encode(p_apneas, accelx, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_apneas->accelxm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_apneas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        //y char
        len     = hrm_encode(p_apneas, accely, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_apneas->accelym_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_apneas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        //z char
        len     = hrm_encode(p_apneas, accelz, encoded_hrm);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_apneas->accelzm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_hrm;

        err_code = sd_ble_gatts_hvx(p_apneas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


void ble_apneas_rr_interval_add(ble_apneas_t * p_apneas, uint16_t rr_interval)
{
    if (p_apneas->rr_interval_count == BLE_APNEAS_MAX_BUFFERED_RR_INTERVALS)
    {
        // The rr_interval buffer is full, delete the oldest value
        memmove(&p_apneas->rr_interval[0],
                &p_apneas->rr_interval[1],
                (BLE_APNEAS_MAX_BUFFERED_RR_INTERVALS - 1) * sizeof(uint16_t));
        p_apneas->rr_interval_count--;
    }

    // Add new value
    p_apneas->rr_interval[p_apneas->rr_interval_count++] = rr_interval;
}


bool ble_apneas_rr_interval_buffer_is_full(ble_apneas_t * p_apneas)
{
    return (p_apneas->rr_interval_count == BLE_APNEAS_MAX_BUFFERED_RR_INTERVALS);
}


uint32_t ble_apneas_sensor_contact_supported_set(ble_apneas_t * p_apneas, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_apneas->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_apneas->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}


void ble_apneas_sensor_contact_detected_update(ble_apneas_t * p_apneas, bool is_sensor_contact_detected)
{
    p_apneas->is_sensor_contact_detected = is_sensor_contact_detected;
}


void ble_apneas_on_gatt_evt(ble_apneas_t * p_apneas, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_apneas->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_apneas->max_hrm_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_APNEAS)

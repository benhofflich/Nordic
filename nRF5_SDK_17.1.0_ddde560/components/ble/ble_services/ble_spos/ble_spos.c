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
#if NRF_MODULE_ENABLED(BLE_SPOS)
#include "ble_spos.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside SpO2 Measurement packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside SpO2 Measurement packet. */
#define MAX_SPOM_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted SpO2 Measurement. */

#define INITIAL_VALUE_SPOM                       0                                    /**< Initial SpO2 Measurement value. */

// SpO2 Measurement flag bits
#define SPOM_FLAG_MASK_SPO2_VALUE_16BIT          (0x01 << 0)                           /**< SpO2 Value Format bit. */
#define SPOM_FLAG_MASK_SENSOR_CONTACT_DETECTED   (0x01 << 1)                           /**< Sensor Contact Detected bit. */
#define SPOM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED  (0x01 << 2)                           /**< Sensor Contact Supported bit. */
#define SPOM_FLAG_MASK_EXPENDED_ENERGY_INCLUDED  (0x01 << 3)                           /**< Energy Expended Status bit. Feature Not Supported */
#define SPOM_FLAG_MASK_RR_INTERVAL_INCLUDED      (0x01 << 4)                           /**< RR-Interval bit. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_spos       SpO2 Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_spos_t * p_spos, ble_evt_t const * p_ble_evt)
{
    p_spos->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_spos       SpO2 Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_spos_t * p_spos, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_spos->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the SpO2 Measurement characteristic.
 *
 * @param[in]   p_spos         SpO2 Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_spom_cccd_write(ble_spos_t * p_spos, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_spos->evt_handler != NULL)
        {
            ble_spos_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_SPOS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_SPOS_EVT_NOTIFICATION_DISABLED;
            }

            p_spos->evt_handler(p_spos, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_spos       SpO2 Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_spos_t * p_spos, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_spos->spom_handles.cccd_handle)
    {
        on_spom_cccd_write(p_spos, p_evt_write);
    }
}


void ble_spos_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_spos_t * p_spos = (ble_spos_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_spos, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_spos, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_spos, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a SpO2 Measurement.
 *
 * @param[in]   p_spos              SpO2 Service structure.
 * @param[in]   spo2         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t spom_encode(ble_spos_t * p_spos, uint16_t spo2, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Set sensor contact related flags
    if (p_spos->is_sensor_contact_supported)
    {
        flags |= SPOM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    }
    if (p_spos->is_sensor_contact_detected)
    {
        flags |= SPOM_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    }

    // Encode SpO2 measurement
    if (spo2 > 0xff)
    {
        flags |= SPOM_FLAG_MASK_SPO2_VALUE_16BIT;
        len   += uint16_encode(spo2, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)spo2;
    }

    // Encode rr_interval values
    if (p_spos->rr_interval_count > 0)
    {
        flags |= SPOM_FLAG_MASK_RR_INTERVAL_INCLUDED;
    }
    for (i = 0; i < p_spos->rr_interval_count; i++)
    {
        if (len + sizeof(uint16_t) > p_spos->max_spom_len)
        {
            // Not all stored rr_interval values can fit into the encoded spom,
            // move the remaining values to the start of the buffer.
            memmove(&p_spos->rr_interval[0],
                    &p_spos->rr_interval[i],
                    (p_spos->rr_interval_count - i) * sizeof(uint16_t));
            break;
        }
        len += uint16_encode(p_spos->rr_interval[i], &p_encoded_buffer[len]);
    }
    p_spos->rr_interval_count -= i;

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}


uint32_t ble_spos_init(ble_spos_t * p_spos, const ble_spos_init_t * p_spos_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_spom[MAX_SPOM_LEN];

    // Initialize service structure
    p_spos->evt_handler                 = p_spos_init->evt_handler;
    p_spos->is_sensor_contact_supported = p_spos_init->is_sensor_contact_supported;
    p_spos->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_spos->is_sensor_contact_detected  = false;
    p_spos->rr_interval_count           = 0;
    p_spos->max_spom_len                 = MAX_SPOM_LEN;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_PLX_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_spos->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add SpO2 measurement characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_PLX_CONTINUOUS_MEAS;
    add_char_params.max_len           = MAX_SPOM_LEN;
    add_char_params.init_len          = spom_encode(p_spos, INITIAL_VALUE_SPOM, encoded_initial_spom);
    add_char_params.p_init_value      = encoded_initial_spom;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_spos_init->spom_cccd_wr_sec;

    err_code = characteristic_add(p_spos->service_handle, &add_char_params, &(p_spos->spom_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_spos_init->p_body_sensor_location != NULL)
    {
        // Add body sensor location characteristic
        memset(&add_char_params, 0, sizeof(add_char_params));

        add_char_params.uuid            = BLE_UUID_BODY_SENSOR_LOCATION_CHAR;
        add_char_params.max_len         = sizeof(uint8_t);
        add_char_params.init_len        = sizeof(uint8_t);
        add_char_params.p_init_value    = p_spos_init->p_body_sensor_location;
        add_char_params.char_props.read = 1;
        add_char_params.char_props.notify = 1;
        add_char_params.read_access     = p_spos_init->bsl_rd_sec;
        add_char_params.cccd_write_access = p_spos_init->spom_cccd_wr_sec;

        err_code = characteristic_add(p_spos->service_handle, &add_char_params, &(p_spos->bsl_handles));
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;
}


uint32_t ble_spos_spo2_measurement_send(ble_spos_t * p_spos, uint16_t spo2)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_spos->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_spom[MAX_SPOM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = spom_encode(p_spos, spo2, encoded_spom);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_spos->spom_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_spom;

        err_code = sd_ble_gatts_hvx(p_spos->conn_handle, &hvx_params);
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


void ble_spos_rr_interval_add(ble_spos_t * p_spos, uint16_t rr_interval)
{
    if (p_spos->rr_interval_count == BLE_SPOS_MAX_BUFFERED_RR_INTERVALS)
    {
        // The rr_interval buffer is full, delete the oldest value
        memmove(&p_spos->rr_interval[0],
                &p_spos->rr_interval[1],
                (BLE_SPOS_MAX_BUFFERED_RR_INTERVALS - 1) * sizeof(uint16_t));
        p_spos->rr_interval_count--;
    }

    // Add new value
    p_spos->rr_interval[p_spos->rr_interval_count++] = rr_interval;
}


bool ble_spos_rr_interval_buffer_is_full(ble_spos_t * p_spos)
{
    return (p_spos->rr_interval_count == BLE_SPOS_MAX_BUFFERED_RR_INTERVALS);
}


uint32_t ble_spos_sensor_contact_supported_set(ble_spos_t * p_spos, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_spos->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_spos->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}


void ble_spos_sensor_contact_detected_update(ble_spos_t * p_spos, bool is_sensor_contact_detected)
{
    p_spos->is_sensor_contact_detected = is_sensor_contact_detected;
}


uint32_t ble_spos_body_sensor_location_set(ble_spos_t * p_spos, uint8_t body_sensor_location)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &body_sensor_location;

    return sd_ble_gatts_value_set(p_spos->conn_handle, p_spos->bsl_handles.value_handle, &gatts_value);
}


void ble_spos_on_gatt_evt(ble_spos_t * p_spos, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_spos->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_spos->max_spom_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_SPOS)

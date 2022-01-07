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
#if NRF_MODULE_ENABLED(BLE_PPGS)
#include "ble_ppgs.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside PPG Measurement packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside PPG Measurement packet. */
#define MAX_PPG_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted PPG Measurement. */

#define INITIAL_VALUE_PPG                       0                                    /**< Initial PPG Measurement value. */

// PPG Measurement flag bits
#define PPG_FLAG_MASK_PPG_VALUE_32BIT          (0x01 << 0)                           /**< PPG Value Format bit. */
#define PPG_FLAG_MASK_SENSOR_CONTACT_DETECTED   (0x01 << 1)                           /**< Sensor Contact Detected bit. */
#define PPG_FLAG_MASK_SENSOR_CONTACT_SUPPORTED  (0x01 << 2)                           /**< Sensor Contact Supported bit. */
#define PPG_FLAG_MASK_EXPENDED_ENERGY_INCLUDED  (0x01 << 3)                           /**< Energy Expended Status bit. Feature Not Supported */
#define PPG_FLAG_MASK_RR_INTERVAL_INCLUDED      (0x01 << 4)                           /**< RR-Interval bit. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ppgs       PPG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ppgs_t * p_ppgs, ble_evt_t const * p_ble_evt)
{
    p_ppgs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ppgs       PPG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ppgs_t * p_ppgs, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ppgs->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the PPG Measurement characteristic.
 *
 * @param[in]   p_ppgs         PPG Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_ppgm_cccd_write(ble_ppgs_t * p_ppgs, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_ppgs->evt_handler != NULL)
        {
            ble_ppgs_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_PPGS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_PPGS_EVT_NOTIFICATION_DISABLED;
            }

            p_ppgs->evt_handler(p_ppgs, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ppgs       PPG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ppgs_t * p_ppgs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_ppgs->rcm_handles.cccd_handle || p_evt_write->handle == p_ppgs->irm_handles.cccd_handle)
    {
        on_ppgm_cccd_write(p_ppgs, p_evt_write);
    }
}


void ble_ppgs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_ppgs_t * p_ppgs = (ble_ppgs_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ppgs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ppgs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ppgs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for encoding a PPG Measurement.
 *
 * @param[in]   p_ppgs              PPG Service structure.
 * @param[in]   ppg         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t ppgm_encode(ble_ppgs_t * p_ppgs, uint32_t ppg, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Set sensor contact related flags
    if (p_ppgs->is_sensor_contact_supported)
    {
        flags |= PPG_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    }
    if (p_ppgs->is_sensor_contact_detected)
    {
        flags |= PPG_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    }

    // Encode PPG measurement
    if (ppg > 0xff)
    {
        flags |= PPG_FLAG_MASK_PPG_VALUE_32BIT;
        len   += uint32_encode(ppg, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)ppg;
    }

    // Encode rr_interval values
    if (p_ppgs->rr_interval_count > 0)
    {
        flags |= PPG_FLAG_MASK_RR_INTERVAL_INCLUDED;
    }
    for (i = 0; i < p_ppgs->rr_interval_count; i++)
    {
        if (len + sizeof(uint16_t) > p_ppgs->max_ppgm_len)
        {
            // Not all stored rr_interval values can fit into the encoded ppgm,
            // move the remaining values to the start of the buffer.
            memmove(&p_ppgs->rr_interval[0],
                    &p_ppgs->rr_interval[i],
                    (p_ppgs->rr_interval_count - i) * sizeof(uint16_t));
            break;
        }
        len += uint16_encode(p_ppgs->rr_interval[i], &p_encoded_buffer[len]);
    }
    p_ppgs->rr_interval_count -= i;

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}

static uint32_t rc_char_add(ble_ppgs_t * p_ppgs, const ble_ppgs_init_t * p_ppgs_init)
{
    uint32_t   err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read          = 0;
    char_md.char_props.write_wo_resp = 0;
    char_md.char_props.notify        = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    ble_uuid128_t base_uuid = {BLE_UUID_RED_COUNTER_CHAR_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ppgs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_ppgs->uuid_type;
    ble_uuid.uuid = BLE_UUID_RED_COUNTER_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    // Configure the characteristic value
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_PPG_LEN;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_ppgs->service_handle, &char_md,
                                       &attr_char_value,
                                       &p_ppgs->rcm_handles);
}

static uint32_t ir_char_add(ble_ppgs_t * p_ppgs, const ble_ppgs_init_t * p_ppgs_init)
{
    uint32_t   err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read          = 0;
    char_md.char_props.write_wo_resp = 0;
    char_md.char_props.notify        = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    ble_uuid128_t base_uuid = {BLE_UUID_INFRARED_CHAR_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ppgs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_ppgs->uuid_type;
    ble_uuid.uuid = BLE_UUID_INFRARED_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    // Configure the characteristic value
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = MAX_PPG_LEN;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_ppgs->service_handle, &char_md,
                                       &attr_char_value,
                                       &p_ppgs->irm_handles);
}

uint32_t ble_ppgs_init(ble_ppgs_t * p_ppgs, const ble_ppgs_init_t * p_ppgs_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_ppgm[MAX_PPG_LEN];

    // Initialize service structure
    p_ppgs->evt_handler                 = p_ppgs_init->evt_handler;
    p_ppgs->is_sensor_contact_supported = p_ppgs_init->is_sensor_contact_supported;
    p_ppgs->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_ppgs->is_sensor_contact_detected  = false;
    p_ppgs->rr_interval_count           = 0;
    p_ppgs->max_ppgm_len                 = MAX_PPG_LEN;

    // Add service
    ble_uuid128_t base_uuid = {BLE_UUID_PPG_SERVICE_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ppgs->uuid_type);

    ble_uuid.type = p_ppgs->uuid_type;
    ble_uuid.uuid = BLE_UUID_PPG_SERVICE_UUID;


    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_ppgs->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = rc_char_add(p_ppgs, p_ppgs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = ir_char_add(p_ppgs, p_ppgs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_ppgs_ir_measurement_send(ble_ppgs_t * p_ppgs, uint32_t infrared)
{
    uint32_t err_code;

    if (p_ppgs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err_code = NRF_SUCCESS;

    // Send value if connected and notifying
    if (p_ppgs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("Infrared: %d\r\n",infrared);
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        uint8_t irData[4];
        len = sizeof(irData);
        irData[0] = (uint8_t)(infrared>>24);
        irData[1] = (uint8_t)(infrared>>16);
        irData[2] = (uint8_t)(infrared>>8);
        irData[3] = (uint8_t)(infrared>>0);

        hvx_len = len;

        ble_gatts_value_t gatts_value;

        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = irData;

         // Update database.
        err_code = sd_ble_gatts_value_set(p_ppgs->conn_handle,
                                          p_ppgs->irm_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ppgs->irm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = irData;

        err_code = sd_ble_gatts_hvx(p_ppgs->conn_handle, &hvx_params);
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

uint32_t ble_ppgs_rc_measurement_send(ble_ppgs_t * p_ppgs, uint32_t redcounter)
{
    uint32_t err_code;

    if (p_ppgs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err_code = NRF_SUCCESS;

    // Send value if connected and notifying
    if (p_ppgs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("Red: %d\r\n",redcounter);
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        uint8_t rcData[4];
        len = sizeof(rcData);
        rcData[0] = (uint8_t)(redcounter>>24);
        rcData[1] = (uint8_t)(redcounter>>16);
        rcData[2] = (uint8_t)(redcounter>>8);
        rcData[3] = (uint8_t)(redcounter>>0);

        hvx_len = len;

        ble_gatts_value_t gatts_value;

        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = rcData;

         // Update database.
        err_code = sd_ble_gatts_value_set(p_ppgs->conn_handle,
                                          p_ppgs->rcm_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ppgs->rcm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = rcData;

        err_code = sd_ble_gatts_hvx(p_ppgs->conn_handle, &hvx_params);
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


void ble_ppgs_rr_interval_add(ble_ppgs_t * p_ppgs, uint16_t rr_interval)
{
    if (p_ppgs->rr_interval_count == BLE_PPGS_MAX_BUFFERED_RR_INTERVALS)
    {
        // The rr_interval buffer is full, delete the oldest value
        memmove(&p_ppgs->rr_interval[0],
                &p_ppgs->rr_interval[1],
                (BLE_PPGS_MAX_BUFFERED_RR_INTERVALS - 1) * sizeof(uint16_t));
        p_ppgs->rr_interval_count--;
    }

    // Add new value
    p_ppgs->rr_interval[p_ppgs->rr_interval_count++] = rr_interval;
}


bool ble_ppgs_rr_interval_buffer_is_full(ble_ppgs_t * p_ppgs)
{
    return (p_ppgs->rr_interval_count == BLE_PPGS_MAX_BUFFERED_RR_INTERVALS);
}


uint32_t ble_ppgs_sensor_contact_supported_set(ble_ppgs_t * p_ppgs, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_ppgs->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_ppgs->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}


void ble_ppgs_sensor_contact_detected_update(ble_ppgs_t * p_ppgs, bool is_sensor_contact_detected)
{
    p_ppgs->is_sensor_contact_detected = is_sensor_contact_detected;
}


uint32_t ble_ppgs_body_sensor_location_set(ble_ppgs_t * p_ppgs, uint8_t body_sensor_location)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &body_sensor_location;

    return sd_ble_gatts_value_set(p_ppgs->conn_handle, p_ppgs->bsl_handles.value_handle, &gatts_value);
}


void ble_ppgs_on_gatt_evt(ble_ppgs_t * p_ppgs, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_ppgs->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_ppgs->max_ppgm_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_PPGS)

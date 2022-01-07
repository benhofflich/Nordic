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
#if NRF_MODULE_ENABLED(BLE_ACCS)
#include "ble_accs.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside ACC Measurement packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside ACC Measurement packet. */
#define MAX_ACC_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted ACC Measurement. */

#define INITIAL_VALUE_ACC                       0                                    /**< Initial ACC Measurement value. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_accs       ACC Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_accs_t * p_accs, ble_evt_t const * p_ble_evt)
{
    p_accs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_accs       ACC Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_accs_t * p_accs, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_accs->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the ACC Measurement characteristic.
 *
 * @param[in]   p_accs         ACC Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_accm_cccd_write(ble_accs_t * p_accs, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_accs->evt_handler != NULL)
        {
            ble_accs_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ACCS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ACCS_EVT_NOTIFICATION_DISABLED;
            }

            p_accs->evt_handler(p_accs, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_accs       ACC Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_accs_t * p_accs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_accs->accxm_handles.cccd_handle | p_evt_write->handle == p_accs->accym_handles.cccd_handle | p_evt_write->handle == p_accs->acczm_handles.cccd_handle)
    {
        on_accm_cccd_write(p_accs, p_evt_write);
    }
}


void ble_accs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_accs_t * p_accs = (ble_accs_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_accs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_accs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_accs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t acc_x_char_add(ble_accs_t * p_accs, const ble_accs_init_t * p_accs_init)
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

    ble_uuid128_t base_uuid = {BLE_UUID_ACCELEROMETER_X_CHAR_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_accs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_accs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACCELEROMETER_X_CHAR_UUID;

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
    attr_char_value.max_len      = MAX_ACC_LEN;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_accs->service_handle, &char_md,
                                       &attr_char_value,
                                       &p_accs->accxm_handles);
}

static uint32_t acc_y_char_add(ble_accs_t * p_accs, const ble_accs_init_t * p_accs_init)
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

    ble_uuid128_t base_uuid = {BLE_UUID_ACCELEROMETER_Y_CHAR_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_accs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_accs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACCELEROMETER_Y_CHAR_UUID;

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
    attr_char_value.max_len      = MAX_ACC_LEN;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_accs->service_handle, &char_md,
                                       &attr_char_value,
                                       &p_accs->accym_handles);
}

static uint32_t acc_z_char_add(ble_accs_t * p_accs, const ble_accs_init_t * p_accs_init)
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

    ble_uuid128_t base_uuid = {BLE_UUID_ACCELEROMETER_Z_CHAR_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_accs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_accs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACCELEROMETER_Z_CHAR_UUID;

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
    attr_char_value.max_len      = MAX_ACC_LEN;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_accs->service_handle, &char_md,
                                       &attr_char_value,
                                       &p_accs->acczm_handles);
}

uint32_t ble_accs_init(ble_accs_t * p_accs, const ble_accs_init_t * p_accs_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_accm[MAX_ACC_LEN];

    // Initialize service structure
    p_accs->evt_handler                 = p_accs_init->evt_handler;
    p_accs->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_accs->max_accm_len                 = MAX_ACC_LEN;

    // Add service
    ble_uuid128_t base_uuid = {BLE_UUID_ACC_SERVICE_BASE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_accs->uuid_type);

    ble_uuid.type = p_accs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ACC_SERVICE_UUID;


    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_accs->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = acc_x_char_add(p_accs, p_accs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = acc_y_char_add(p_accs, p_accs_init);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_DEBUG("Y: %d",err_code);
        return err_code;
    }

    err_code = acc_z_char_add(p_accs, p_accs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_accs_acc_measurement_send(ble_accs_t * p_accs, uint16_t xAcc, uint16_t yAcc, uint16_t zAcc)
{
    uint32_t err_code;

    if (p_accs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err_code = NRF_SUCCESS;

    // Send value if connected and notifying
    if (p_accs->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        uint8_t xAccData[2];
        uint8_t yAccData[2];
        uint8_t zAccData[2];
        len = sizeof(xAccData);

        xAccData[0] = (uint8_t)(xAcc>>8);
        xAccData[1] = (uint8_t)(xAcc>>0);

        yAccData[0] = (uint8_t)(yAcc>>8);
        yAccData[1] = (uint8_t)(yAcc>>0);

        zAccData[0] = (uint8_t)(zAcc>>8);
        zAccData[1] = (uint8_t)(zAcc>>0);

        hvx_len = len;

        ble_gatts_value_t gatts_value;

        // Initialize X value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = xAccData;

         // Update database.
        err_code = sd_ble_gatts_value_set(p_accs->conn_handle,
                                          p_accs->accxm_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        NRF_LOG_INFO("X: %d\r\n", (int16_t)xAcc);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accs->accxm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = xAccData;

        err_code = sd_ble_gatts_hvx(p_accs->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        // Initialize Y value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = yAccData;

         // Update database.
        err_code = sd_ble_gatts_value_set(p_accs->conn_handle,
                                          p_accs->accym_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        NRF_LOG_INFO("Y: %d\r\n", (int16_t)yAcc);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accs->accym_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = yAccData;

        err_code = sd_ble_gatts_hvx(p_accs->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        // Initialize Z value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = zAccData;

         // Update database.
        err_code = sd_ble_gatts_value_set(p_accs->conn_handle,
                                          p_accs->acczm_handles.value_handle,
                                          &gatts_value);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        NRF_LOG_INFO("Z: %d\r\n", (int16_t)zAcc);

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_accs->acczm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = zAccData;

        err_code = sd_ble_gatts_hvx(p_accs->conn_handle, &hvx_params);
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


void ble_accs_on_gatt_evt(ble_accs_t * p_accs, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_accs->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_accs->max_accm_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_ACCS)

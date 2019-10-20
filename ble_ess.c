/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
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
#if NRF_MODULE_ENABLED(BLE_ESS)
#include "ble_ess.h"
#include <string.h>
#include "ble_srv_common.h"
#include "ble_conn_state.h"
#include "DHT22.h"

#define NRF_LOG_MODULE_NAME ble_ess
#if BLE_BAS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_BAS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_BAS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_BAS_CONFIG_DEBUG_COLOR
#else // BLE_BAS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BLE_BAS_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#define INITIAL_VALUE_TEMPERATURE 0.0


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    p_ess->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ess->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling write events to the Temperature characteristic.
 *
 * @param[in]   p_ess         Environmental Sensing Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_temperature_cccd_write(ble_ess_t * p_ess, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_ess->evt_handler != NULL)
        {
            ble_ess_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_DISABLED;
            }

            p_ess->evt_handler(p_ess, &evt);
        }
    }
}

/**@brief Function for handling write events to the Humidity characteristic.
 *
 * @param[in]   p_ess         Environmental Sensing Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_humidity_cccd_write(ble_ess_t * p_ess, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_ess->evt_handler != NULL)
        {
            ble_ess_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_ESS_EVT_NOTIFICATION_DISABLED;
            }

            p_ess->evt_handler(p_ess, &evt);
        }
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ess_t * p_ess, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_ess->temperature_handles.cccd_handle)
    {
        on_temperature_cccd_write(p_ess, p_evt_write);
    }
    if (p_evt_write->handle == p_ess->humidity_handles.cccd_handle)
    {
        on_humidity_cccd_write(p_ess, p_evt_write);
    }
}

void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_ess_t * p_ess = (ble_ess_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ess, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ess, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ess, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


static uint8_t temperature_encode(uint8_t * p_encoded_buffer)
{
    // Encode temperature
    // R = C * M * 10^d * 2^b ----- M = 1, d = -2, b = 0
    // We have R and need to get C to transmit
    // C = R / (M * 10^d * 2^b)
    // C = temperature / (1 * 1/100 * 1)
    // C = temperature * 100

    read_dht22_dat();

    return uint16_encode(getTemp() * 10, &p_encoded_buffer[0]);

}

static uint8_t humidity_encode(uint8_t * p_encoded_buffer)
{
    // Encode humidity
    // R = C * M * 10^d * 2^b ----- M = 1, d = -2, b = 0
    // We have R and need to get C to transmit
    // C = R / (M * 10^d * 2^b)
    // C = temperature / (1 * 1/100 * 1)
    // C = temperature * 100

    read_dht22_dat();

    return uint16_encode(getHumid() * 10, &p_encoded_buffer[0]);

}

ret_code_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_temperature[2];
    uint8_t               encoded_initial_humidity[2];

    // Initialize service structure
    p_ess->evt_handler                 = p_ess_init->evt_handler;
    p_ess->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_ess->max_temperature_len         = 2;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_ess->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add temperature characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_TEMPERATURE_CHAR;
    add_char_params.max_len           = 2;
    add_char_params.init_len          = temperature_encode(encoded_initial_temperature);
    add_char_params.p_init_value      = encoded_initial_temperature;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_ess_init->temp_cccd_wr_sec;

    err_code = characteristic_add(p_ess->service_handle, &add_char_params, &(p_ess->temperature_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add humidity characteristic
    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid              = BLE_UUID_HUMIDITY_CHAR;
    add_char_params.max_len           = 2;
    add_char_params.init_len          = humidity_encode(encoded_initial_humidity);
    add_char_params.p_init_value      = encoded_initial_humidity;
    add_char_params.is_var_len        = true;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = p_ess_init->humid_cccd_wr_sec;

    err_code = characteristic_add(p_ess->service_handle, &add_char_params, &(p_ess->humidity_handles));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_ess_temperature_send(ble_ess_t * p_ess)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_ess->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_temperature[2];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = temperature_encode(encoded_temperature);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ess->temperature_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_temperature;

        err_code = sd_ble_gatts_hvx(p_ess->conn_handle, &hvx_params);
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

uint32_t ble_ess_humidity_send(ble_ess_t * p_ess)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_ess->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_humidity[2];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = humidity_encode(encoded_humidity);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ess->humidity_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_humidity;

        err_code = sd_ble_gatts_hvx(p_ess->conn_handle, &hvx_params);
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

#endif // NRF_MODULE_ENABLED(BLE_ESS)

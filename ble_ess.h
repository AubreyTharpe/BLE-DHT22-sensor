/** @file
 *
 * @defgroup ble_ess Environmental Sensing Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Environmental Sensing Service module.
 *
 * @details This module implements the Environmental Sensing Service with the Temperature and Humidity
 *          characteristic.
 *          During initialization it adds the Environmental Sensing Service, Temperature, and Humidity
 *          characteristic to the BLE stack database.
 *
 */
#ifndef BLE_ESS_H__
#define BLE_ESS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Macro for defining a ble_ess instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_ESS_DEF(_name)                          \
    static ble_ess_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                         BLE_BAS_BLE_OBSERVER_PRIO, \
                         ble_ess_on_ble_evt,        \
                         &_name)
#define BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE    0x181A                        /**< Environmental Sensing service UUID. */
#define BLE_UUID_HUMIDITY_CHAR                    0x2A6F                        /**< Humidity characteristic UUID. */
#define BLE_UUID_TEMPERATURE_CHAR                 0x2A6E                        /**< Temperature characteristic UUID. */

/**@brief Environmental Sensing Service event type. */
typedef enum
{
    BLE_ESS_EVT_NOTIFICATION_ENABLED,
    BLE_ESS_EVT_NOTIFICATION_DISABLED
} ble_ess_evt_type_t;

/**@brief Environmental Sensing Service event. */
typedef struct
{
    ble_ess_evt_type_t evt_type;    /**< Type of event. */
    uint16_t           conn_handle; /**< Connection handle. */
} ble_ess_evt_t;

// Forward declaration of the ble_ess_t type.
typedef struct ble_ess_s ble_ess_t;

/**@brief Environmental Sensing Service event handler type. */
typedef void (* ble_ess_evt_handler_t) (ble_ess_t * p_ess, ble_ess_evt_t * p_evt);

/**@brief Environmental Sensing Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_ess_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Environmental Sensing Service. */
    bool                   support_notification;
    ble_srv_report_ref_t * p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    security_req_t         temp_cccd_wr_sec;               /**< Security requirement for writing the temperature characteristic CCCD. */
    security_req_t         humid_cccd_wr_sec;              /**< Security requirement for writing the humidity characteristic CCCD. */
} ble_ess_init_t;

/**@brief Environmental Sensing Service structure. This contains various status information for the service. */
struct ble_ess_s
{
    ble_ess_evt_handler_t    evt_handler;               /**< Event handler to be called for handling events in the Environmental Sensing Service. */
    uint16_t                 service_handle;            /**< Handle of Environmental Sensing Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t temperature_handles;       /**< Handles related to the Temperature characteristic. */
    ble_gatts_char_handles_t humidity_handles;          /**< Handles related to the Humidity characteristic. */
    uint16_t                 conn_handle;               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                     is_notification_supported;
    uint8_t                  max_temperature_len;
};


/**@brief Function for initializing the Environmental Sensing Service.
 *
 * @param[out]  p_ess       Environmental Sensing Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_ess_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
ret_code_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Environmental Sensing Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Environmental Sensing Service structure.
 */
void ble_ess_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

uint32_t ble_ess_temperature_send(ble_ess_t * p_ess);

uint32_t ble_ess_humidity_send(ble_ess_t * p_ess);

#ifdef __cplusplus
}
#endif

#endif // BLE_ESS_H__

/** @} */

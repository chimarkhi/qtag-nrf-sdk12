#ifndef OUR_SERVICE_H__
#define OUR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_TBS_BASE              {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define BLE_UUID_TBS                		0xAB2F			 // TagBox service UUID
#define BLE_UUID_TBS_CHAR_LOGINTERVAL	  0xAB30 			 // Logging interval char UUID
#define BLE_UUID_TBS_CHAR_ADVINTERVAL	  0xAB31 			 // Advertising power char UUID
#define BLE_UUID_TBS_CHAR_TXPOWER			  0xAB32 			 // Tx power char UUID

// This structure contains various status information for tagbox service. 
typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of tbs Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    char_handles;		/**< handles for the characteristic attributes to our struct*/
}ble_tbs_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_tbs_on_ble_evt(ble_tbs_t * p_our_service, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */


/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
uint32_t ble_tbs_init(ble_tbs_t * p_our_service);


/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void tbs_txpower_characteristic_update(ble_tbs_t *p_our_service, int32_t *txpower_value);

uint8_t ble_tbs_get_advinterval(ble_tbs_t * p_tbs);

#endif /* _ OUR_SERVICE_H__ */

#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "ble_tbs.h"
#include "ble_srv_common.h"
#include "app_error.h"

#define  NRF_LOG_MODULE_NAME 						"BLE_TBS"
#include "nrf_log.h"

// Declaration of a function for housekeeping of ble connections related to tbs service and characteristic
void ble_tbs_on_ble_evt(ble_tbs_t * p_tbs, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
{
    case BLE_GAP_EVT_CONNECTED:
        p_tbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        p_tbs->conn_handle = BLE_CONN_HANDLE_INVALID;
        break;
    default:
        // No implementation needed.
        break;
}
}

/**@brief Function for adding our new characterstic to TagBox Service 
 *
 * @param[in]   p_tbs        tagbox service structure.
 *
 */
static uint32_t tbs_char_add(ble_tbs_t * p_tbs, uint16_t char_uuid16, uint8_t char_value_init)
{
    // Add a custom characteristic UUID
		uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = BLE_UUID_TBS_BASE;
		char_uuid.uuid      = char_uuid16;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		if (err_code != NRF_SUCCESS) return err_code; 
    
		// Set up properties to characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    //ble_gatts_attr_md_t cccd_md;
    //memset(&cccd_md, 0, sizeof(cccd_md));
   
    
    //  Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));  
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    
    // Set read/write security levels to characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    //  Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
	attr_char_value.p_attr_md   = &attr_md;

    // Set characteristic length in number of bytes
		attr_char_value.max_len     	= 1;
		attr_char_value.init_len    	= 1;
		uint8_t value[1] 							= {char_value_init};
		attr_char_value.p_value     	= value;

    // Add new characteristic to the service
		err_code = sd_ble_gatts_characteristic_add(p_tbs->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_tbs->char_handles);
		if (err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
uint32_t ble_tbs_init(ble_tbs_t * p_tbs)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // FROM_SERVICE_TUTORIAL: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_TBS_BASE;
    service_uuid.uuid = BLE_UUID_TBS;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    if (err_code != NRF_SUCCESS) return err_code;

	
    // Set tbs service connection handle to default value, i.e. an invalid handle at system startup
		p_tbs->conn_handle = BLE_CONN_HANDLE_INVALID;
	
    // Add tbs service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_tbs->service_handle);
    
    if (err_code != NRF_SUCCESS) return err_code;
    
    // add new characteristics to the service. 
		err_code = tbs_char_add(p_tbs, BLE_UUID_TBS_CHAR_LOGINTERVAL, 30);
    if (err_code != NRF_SUCCESS) return err_code;
		err_code = tbs_char_add(p_tbs, BLE_UUID_TBS_CHAR_ADVINTERVAL, 30);
    if (err_code != NRF_SUCCESS) return err_code;
		err_code = tbs_char_add(p_tbs, BLE_UUID_TBS_CHAR_TXPOWER, 30);
		if (err_code != NRF_SUCCESS) return NRF_SUCCESS;
		
		
		return NRF_SUCCESS;
}

// Function to be called when updating characteristic value
void tbs_txpower_characteristic_update(ble_tbs_t *p_our_service, int32_t *temperature_value)
{
    // OUR_JOB: Step 3.E, Update characteristic value

}

// Unused
uint8_t ble_tbs_get_advinterval(ble_tbs_t * p_tbs)
{
	uint8_t data_buf;
	ble_gatts_value_t gatts_buf;
	gatts_buf.len = 1; 
	gatts_buf.offset = 0; 
	gatts_buf.p_value = &data_buf;
	
	uint32_t err_code = sd_ble_gatts_value_get(BLE_CONN_HANDLE_INVALID, p_tbs->char_handles.value_handle, &gatts_buf );
	//NRF_LOG_DEBUG("char read err %d, value %d",err_code, &gatts_buf.p_value[0]);
	
	return data_buf;
}

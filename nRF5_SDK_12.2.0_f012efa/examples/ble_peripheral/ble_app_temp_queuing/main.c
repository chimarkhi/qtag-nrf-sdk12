/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "main.h"

#define  NRF_LOG_MODULE_NAME 			"APP"
#include "nrf_log.h"
#include "advertising.h"
#include "bledb.h"
#include "ble_dfu.h"

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
//static ble_tbs_t						m_tbs;																			/**< Structure to identify the TagBox Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static nrf_drv_twi_t 					twi = NRF_DRV_TWI_INSTANCE(0);
static ble_dfu_t m_dfus;                                            /**< Structure used to identify the DFU service. */



// Global variables
uint16_t nusRecKey;
uint16_t nusCurrentKey;
uint32_t tstamp_sec;

volatile advertising_mode_t advMode;
volatile bool initFlag = false;
volatile bool gcDone = false;
volatile bool writeFlag = false;
sync_type_t syncType = SYNCTYPE_STRAIGHT;
bool isNUSClient = false;

char strDeviceName[20];
char strDeviceName_temp[20];

APP_TIMER_DEF(m_advdata_update_timer);
APP_TIMER_DEF(m_dataToDB_timer);
APP_TIMER_DEF(m_tstamp_timer);

/***@brief function for initializing nrf logging
*/
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}

void advdata_update_timer_timeout_handler(void * p_context)
{
	advdata_update(advMode);
#ifndef BB_DEVICE
	indicate_advertising(advMode);
#endif
}

void tstamp_timer_timeout_handler(void * p_context)
{
	++tstamp_sec;
}

uint32_t get_timeStamp(void)
{
	return tstamp_sec;
}

void device_name_set(void)
{
        uint32_t macid_dec;

        macid_dec = NRF_FICR->DEVICEADDR0%100000;
        macid_dec = NRF_FICR->DEVICEADDR0;



        sprintf(&strDeviceName_temp[0], "B1%x", macid_dec);

        strDeviceName[0] = 'B';
        strDeviceName[1] = 'B';
        for(int i=2;i<=5;i++)
        {
         strDeviceName[i] = strDeviceName_temp[i+4];

        /*      if(strDeviceName_temp[i+4]>96 && strDeviceName_temp[i+4]<122)
                {strDeviceName[i]=strDeviceName[i+4]-32;}
                else
                {strDeviceName[i]=strDeviceName[i+4];}
        */
        };


        NRF_LOG_INFO("Str Name: %s\n",strDeviceName );
        NRF_LOG_INFO("Str_temp Name: %s\n",strDeviceName_temp );


        //DEVICE_NAME = "Test";
        NRF_LOG_INFO("\r\nMacid value %d\n", macid_dec );
        NRF_LOG_INFO("\r\nMacid value 0x%02x\n", macid_dec );
        NRF_LOG_INFO("Device Name: %s\n",DEVICE_NAME );
}

/**@brief Function for the tstamp and record key
 *
 * @details Initializes reckey and tstamp from last record key and timestamp stored in REC_KEY_LASTSEEN.
 * @details a check on read data's reckey is made, to see if it is within expected range. Otherwise tstamp 
 * @details and recCounter initialized to 0
 * @details Should be called only on device reset (battery change)
 */
void tstamp_reckey_init()
{
		uint32_t data[] 	= {0,0,0};
		uint8_t dataLen 	= sizeof(data);
		uint32_t err_code;
		err_code = fds_read(BACKUP_FILE_ID, REC_KEY_LASTSEEN, data, &dataLen);
		if (err_code != NRF_SUCCESS) NRF_LOG_ERROR("FDS read error %d", err_code);
		
		if ((data[0] <= (uint32_t)0xFFFF)
			&& (data[0] >= (uint32_t)REC_KEY_START )
			&& (err_code == FDS_SUCCESS))
		{
			tstamp_sec = data[1]+1;
			recCounter_init((uint16_t)data[0]+1);
		}
		else
		{
			tstamp_sec = 0;
			recCounter_init(REC_KEY_START);
		}
		
		NRF_LOG_INFO("Starting from RECKEY,tstamp: [%04x,%08x]\r\n",(uint16_t)data[0],tstamp_sec);
}

/**@brief Function to get internal temperature of IC. Used for calibration */
static uint32_t get_internal_temp(void)
{
    int32_t temp;
    uint32_t err_code;

    err_code = sd_temp_get(&temp);
    //NRF_LOG_ERROR("Internal temp get error : %d",err_code);

    temp = temp*25; // -200 for Minew_Tag16 and 07; -425 for Minew_tag13;
    int8_t exponent = -2;
    return ((exponent & 0xFF) << 24) | (temp & 0x00FFFFFF);
}

uint32_t get_telemetry_data(uint16_t* p_temp, uint8_t* p_humid, uint8_t* p_batt_level,
								uint32_t* p_timeStamp, uint16_t* p_recKey)
{
		uint32_t err_code = NRF_SUCCESS;
		
		#ifdef BB_DEVICE
			*p_temp	= get_internal_temp();
			*p_humid 	= 47;
			*p_batt_level = get_battery_level();
		#else	
			err_code 	= get_temp_humid(&twi, p_temp, p_humid);
			*p_batt_level = get_battery_level();
		#endif
	
 		if (err_code != NRF_SUCCESS)	return err_code;
		
		// Add logic to do data sanity checks on temperature, humidity and battery
		
		*p_timeStamp = tstamp_sec;
		*p_recKey = get_recKey();
		
		return NRF_SUCCESS;
}


void dataToDB_timer_timeout_handler(void * p_context)
{
		uint32_t	timeStamp;
		uint16_t 	temp;
		uint8_t 	humid, batt_level;
		uint16_t 	recKey;
		uint32_t err_code;
		uint32_t dataPacket[WORDLEN_DATAPACKET];
		uint8_t i ;

		for (i=0; i<NUMBER_SHT31_TRIES; i++){
			err_code = get_telemetry_data(&temp, &humid, &batt_level, &timeStamp, &recKey);
			if (err_code == NRF_SUCCESS) break;	// Break out of loop @ clean data capture
			else NRF_LOG_ERROR("Invalid SHT Data");
		}

		dataPacket[0]	=	recKey;
		dataPacket[1]	=	timeStamp;

		if (err_code != NRF_SUCCESS) {			// In case of invalid data fill in the previous saved data point
			uint8_t dataLen;
			uint32_t data[3] = {0,0,0};
			err_code = fds_read(BACKUP_FILE_ID, REC_KEY_LASTSEEN, data, &dataLen);
			dataPacket[2] = data[dataLen - 1];
		}
		else dataPacket[2] = ((temp << 16) | (humid << 8) | batt_level);

		NRF_LOG_RAW_INFO("\r\n\n\n");
		NRF_LOG_DEBUG("Data to DB: RecKey %08x, time %08x, data %08x\r\n", dataPacket[0], dataPacket[1], dataPacket[2]);
		err_code = dataToDB(FILE_ID, recKey, dataPacket, WORDLEN_DATAPACKET);
		// if data saved successfully, update the last seen reckey and tstamp in flash												 

		if (err_code == NRF_SUCCESS) 
		{
//			wait_for_fds_evt(FDS_EVT_WRITE);
			err_code = fds_find_and_delete(BACKUP_FILE_ID, REC_KEY_LASTSEEN);
			err_code = fds_write(BACKUP_FILE_ID, REC_KEY_LASTSEEN, dataPacket, WORDLEN_DATAPACKET);
		}
		else	NRF_LOG_ERROR("DataToDB error : %d", err_code); 
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    uint32_t err_code = app_timer_create(&m_advdata_update_timer, APP_TIMER_MODE_REPEATED, advdata_update_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_dataToDB_timer, APP_TIMER_MODE_REPEATED, dataToDB_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_tstamp_timer, APP_TIMER_MODE_REPEATED, tstamp_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);	

}


/**@brief Function for starting timers.
*/
static void timers_start(void)
{
	uint32_t err_code ;
	//err_code = app_timer_start(m_advdata_update_timer, ADVDATA_UPDATE_INTERVAL, NULL);
	//APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_dataToDB_timer, LOGINTERVAL_ADVINTERVAL_RATIO*ADVDATA_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_tstamp_timer, TSTAMP_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief 	Function for reloading advdata update timer with new timeout value
	 @details	In case of failure restart the timer with default value
	 @input 	timeout_ticks: 	new value to be loaded
	 @input		stopTimer : function return without restarting the timer with new timeout value
*/
uint32_t dynadv_timer_update(uint32_t timeoutTicks)
{
	uint32_t err_code = app_timer_stop(m_advdata_update_timer);

	// timer_stop will return invalid_state error when function is called from ADV_MODE_OFF
	if (err_code == NRF_SUCCESS)
	{
		err_code = app_timer_start(m_advdata_update_timer, timeoutTicks, NULL);
		NRF_LOG_DEBUG("Adv timer updated with NEW timeout value: %d\r\n", timeoutTicks);
	}
	else
	{
		NRF_LOG_ERROR("Adv timer could not be started error: %d\r\n", err_code);
	}
	return err_code;
}

uint32_t dynadv_timer_stop(void)
{
	uint32_t err_code = app_timer_stop(m_advdata_update_timer);
	return err_code;

}

uint32_t dynadv_timer_start(uint32_t timeoutTicks)
{
	uint32_t err_code = app_timer_start(m_advdata_update_timer, timeoutTicks,NULL);
	return err_code;

}


static void gap_params_init(void)
{
	uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
        device_name_set();
    err_code = sd_ble_gap_device_name_set(&sec_mode, (uint8_t *) strDeviceName, strlen(strDeviceName));

    if (err_code == NRF_SUCCESS) {
    	NRF_LOG_INFO("Device Name: %s\n",strDeviceName);
    }
    else NRF_LOG_ERROR("GAP Name couldnot be set");
    //APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

}

/**@brief 	Function for handling events from DFU service
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module. **/
static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case BLE_DFU_EVT_INDICATION_DISABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is disabled\r\n");
            break;

        case BLE_DFU_EVT_INDICATION_ENABLED:
            NRF_LOG_INFO("Indication for BLE_DFU is enabled\r\n");
            break;

        case BLE_DFU_EVT_ENTERING_BOOTLOADER:
            NRF_LOG_INFO("Device is entering bootloader mode!\r\n");
            break;
        default:
            NRF_LOG_INFO("Unknown event from ble_dfu\r\n");
            break;
    }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
			NRF_LOG_INFO("Inside nus data handler\r\n");

			NRF_LOG_INFO("data from central (Only 2 bytes used by code):");	
			for (uint32_t i = 0; i < length; i++)
			{
				NRF_LOG_RAW_INFO("%02x",p_data[1-i]);
			}
			NRF_LOG_RAW_INFO("\r\n");
			isNUSClient = true;
			uint16_t inRecKey = p_data[1]<<8|p_data[0];
//			uint16_t inRecKey = (((p_data[0]-0x30)&0x0F)<<12|
//								((p_data[1]-0x30)&0x0F)<<8|
//								((p_data[2]-0x30)&0x0F)<<4|
//								((p_data[3]-0x30)&0x0F));
			NRF_LOG_INFO("Start record : 0x%04x\r\n", inRecKey);
			nusCurrentKey = get_recKey();
			
			// If Record Key sent by TagLink isnt 0x0000 start data tx from appropiate point found using check_startRecKey()
			// else send error message 
			syncType = check_startRecKey(inRecKey, nusCurrentKey);
			if (syncType != SYNCTYPE_INVALID)
			{
				NRF_LOG_INFO("Starting data transfer from %04x, syncType %d\r\n", nusRecKey,syncType);
				payload_to_central_async(&m_nus, nusRecKey);
			}
			else 
			{
				NRF_LOG_WARNING("Invalid input recKey, Sending EOM\r\n\n");
				nus_eom_send(&m_nus,NUS_MSGTYPE_DIRTYRECKEY);
			}
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static uint32_t services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    ble_dfu_init_t dfus_init;

    // Add Nordic Uart Service to GATT
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;
	err_code = ble_nus_init(&m_nus, &nus_init);
    if (err_code != NRF_SUCCESS) return err_code;


    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));
    dfus_init.evt_handler                               = ble_dfu_evt_handler;
    dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
    dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;
    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    if (err_code != NRF_SUCCESS) return err_code;


	// Add TagBox Service to GATT
	//err_code = ble_tbs_init(&m_tbs);
	//if (err_code != NRF_SUCCESS) return err_code;

	return err_code;
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

void sync_handler(void)
{
	uint8_t nus_status;
	uint32_t err_code;
	nusRecKey++;
	nus_status = check_nusRecKey(nusRecKey, nusCurrentKey, syncType);
	if(nus_status == NUS_CONTINUE)	payload_to_central_async(&m_nus, nusRecKey);
	else if (nus_status == NUS_STOP)
	{
		err_code = nus_eom_send(&m_nus, NUS_MSGTYPE_EOM);
		if (err_code != NRF_SUCCESS) {
			NRF_LOG_ERROR("Err sending eom package: %d\r\n", err_code);
		}
		else NRF_LOG_INFO("EOM sent \r\n");
	}
}

/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
	uint8_t nus_status;
	ble_gap_addr_t peerAddress;
	
    switch (p_ble_evt->header.evt_id)
    {
		case BLE_GAP_EVT_SCAN_REQ_REPORT:			// On being scanned by a gateway device
//			if((peerAddress.addr[0] == 0x5C) || (peerAddress.addr[0] == 0x00)) {
				peerAddress = p_ble_evt->evt.gap_evt.params.scan_req_report.peer_addr;
				uint8_t rssiValue = p_ble_evt->evt.gap_evt.params.scan_req_report.rssi;
				NRF_LOG_DEBUG("Advertisement Scanned by GW: %02x:%02x, at RSSI: \r\n",
							peerAddress.addr[4],peerAddress.addr[5], rssiValue);

				advMode = dynamic_advertising_handler(advMode,DYNADV_EVT_GATEWAY_FOUND);
//			}
			break;

		case BLE_GAP_EVT_CONNECTED:
			advMode = dynamic_advertising_handler(advMode,DYNADV_EVT_CONNECTED);
			err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
			m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			nrf_gpio_pin_write(LED_PIN_CONNECTION,1);// Minew S1 v1.0 Red LED
			//nrf_gpio_pin_write(18,1);				// Minew S1 v0.9 Red LED
			break; // BLE_GAP_EVT_CONNECTED

		case BLE_EVT_TX_COMPLETE:					// Send next key event
			NRF_LOG_INFO("TX Complete\r\n");
			sync_handler();
			break; // BLE_EVT_TX_COMPLETE

		case BLE_GAP_EVT_DISCONNECTED:
			nrf_gpio_pin_write(LED_PIN_CONNECTION,0);	// Minew S1 v1.0 Red LED
			//nrf_gpio_pin_write(18,0);					// Minew S1 v0.9 Red LED

			nus_status = check_nusRecKey(nusRecKey, nusCurrentKey, syncType);
			// If eom sent move to unconnectable mode, otherwise to resume fast advertising
			if (!isNUSClient)			//NUS didnt start or sync ended in an unexpected reckey
				advMode = dynamic_advertising_handler(advMode, DYNADV_EVT_CONN_DROP_INVALIDGW);
			else if ((nus_status == NUS_CONTINUE))						//Sync didn't end
				advMode = dynamic_advertising_handler(advMode, DYNADV_EVT_CONN_DROPPED);
			else														//Sync completed
				advMode = dynamic_advertising_handler(advMode, DYNADV_EVT_DATA_SYNCED);

			// reset flags/handles
			syncType = SYNCTYPE_STRAIGHT;
			isNUSClient = false;
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
			break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
               	NRF_LOG_DEBUG("Advertising Timeout\r\n");
            	advMode = dynamic_advertising_handler(advMode, DYNADV_EVT_ADV_TIMEOUT);
            }
            break; // BLE_GAP_EVT_TIMEOUT

		case BLE_EVT_USER_MEM_REQUEST:
			err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
			APP_ERROR_CHECK(err_code);
			break; // BLE_EVT_USER_MEM_REQUEST

		case BLE_GATTS_EVT_WRITE:
		{
			//uint8_t * pdata = p_ble_evt->evt.gatts_evt.params.write.data;
			//ble_uuid_t uuid = p_ble_evt->evt.gatts_evt.params.write.uuid;
			//ble_tbs_get_advinterval(&m_tbs);
		}
			break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

		default:
			// No implementation needed.
			break;
    }
}



static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
//   ble_conn_state_on_ble_evt(p_ble_evt);
//   pm_on_ble_evt(p_ble_evt);
//   ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
	 ble_conn_params_on_ble_evt(p_ble_evt);
	 ble_nus_on_ble_evt(&m_nus, p_ble_evt);
//	 ble_tbs_on_ble_evt(&m_tbs, p_ble_evt);
//   ble_ias_on_ble_evt(&m_ias, p_ble_evt);
//   ble_lls_on_ble_evt(&m_lls, p_ble_evt);
//   ble_bas_on_ble_evt(&m_bas, p_ble_evt);
//   ble_ias_c_on_ble_evt(&m_ias_c, p_ble_evt);
//   ble_tps_on_ble_evt(&m_tps, p_ble_evt);
   bsp_btn_ble_on_ble_evt(p_ble_evt);
   ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
   on_ble_evt(p_ble_evt);
   ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    fs_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
		ble_opt_t ble_options;
	
	
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
	
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
		APP_ERROR_CHECK(err_code);
    ble_enable_params.common_enable_params.vs_uuid_count = 2;
		
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
		APP_ERROR_CHECK(err_code);
		
		// Enable scan response report
		ble_options.gap_opt.scan_req_report.enable = 1;
		err_code = sd_ble_opt_set(BLE_GAP_OPT_SCAN_REQ_REPORT, &ble_options);
		APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
		APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
		APP_ERROR_CHECK(err_code);	
	
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    NRF_LOG_DEBUG("\r\nIn bsp event handler, event : %d\r\n", event);
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;
		case BSP_EVENT_ADVERTISING_START:
			advMode = dynamic_advertising_handler(advMode, DYNADV_EVT_BUTTON_PRESS);
			break;
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;
        default:
            break;
    }
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static uint32_t buttons_leds_init(bool * p_erase_bonds)
{
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
		if(err_code != NRF_SUCCESS) return err_code;


		err_code = bsp_event_to_button_action_assign(ADV_BUTTON, BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_ADVERTISING_START);
		if(err_code != NRF_SUCCESS) return err_code;
	
		return err_code;
}



/**@brief Function for doing power management.
 */
static void power_manage(void)
{
		uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
	uint32_t err_code = NRF_SUCCESS;
	bsp_board_leds_init();
#ifdef BB_DEVICE
	uint8_t i =0;
	for (i=13; i<18; i++){
	    nrf_gpio_cfg(
	        i,
	        NRF_GPIO_PIN_DIR_INPUT,
	        NRF_GPIO_PIN_INPUT_DISCONNECT,
	        NRF_GPIO_PIN_PULLDOWN,
	        NRF_GPIO_PIN_S0S1,
	        NRF_GPIO_PIN_NOSENSE);;
	}
#endif
	nrf_delay_ms(500); // To allow for bounce during battery insertion
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

	// Initialize.
	log_init();
	NRF_LOG_INFO("Starting Logging\n");
	NRF_LOG_INFO("\r\n\n\n\nInitializing ...\n");

	timers_init();
	NRF_LOG_INFO("Timers Initialized \n");

	err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
	if(err_code == NRF_SUCCESS)	{	NRF_LOG_INFO("Initialized BSP \n");}
	else NRF_LOG_ERROR("Error in initialzing BSP rc: %d\r\n", err_code);

	err_code = buttons_leds_init(false);
	if (err_code != NRF_SUCCESS)
	{
		NRF_LOG_ERROR("LEDs init error %d\r\n", err_code);
	}
	else NRF_LOG_INFO("LEDs Initialized \n");

	ble_stack_init();
	gap_params_init();
	NRF_LOG_INFO("ble stack Initialized \n");

#ifndef ADC_PRESENT
	adc_configure();
#endif

	//Initialize advMode to OFF
	advMode = advertising_init();
	NRF_LOG_INFO("Adv Initialized \n");

	// Wait for fds to be initialized before any read/write
	err_code =fds_bledb_init();
	//APP_ERROR_CHECK(err_code);
	// POLL FOR INIT CALLBACK
	wait_for_fds_evt(FDS_EVT_INIT);

	//DFU service initialization requires fs_init to be called first. Done by fds_bledb_init()
	err_code = services_init();
	if (err_code != NRF_SUCCESS) NRF_LOG_ERROR("Services init error %d\r\n", err_code);
	//else NRF_LOG_INFO("Services Initialized \n");

	conn_params_init();
	NRF_LOG_INFO("Conn Params Initialized \n");

	err_code = twi_init(&twi);
	NRF_LOG_INFO("SHT31's I2C Initialized \n");


#ifdef INIT_DEVICE

	NRF_LOG_INFO("\r\n\n----Device Flash Init Begin----\n");

	err_code = fds_file_delete(FILE_ID);
	NRF_LOG_ERROR("file del err: %d \n",err_code);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_RAW_INFO("\r\n\n");
	// Wait for GC to complete on FILE
	wait_for_fds_evt(FDS_EVT_GC);

	err_code = fds_file_delete(BACKUP_FILE_ID);
	NRF_LOG_ERROR("file del err: %d \n",err_code);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_RAW_INFO("\r\n\n");
	// Wait for GC to complete on FILE
	wait_for_fds_evt(FDS_EVT_GC);


	NRF_LOG_INFO("Writing data to LASTSEEN record:\r\n");
	uint32_t testData[] = {(uint32_t)REC_KEY_START,0x00000000,0x00000000};
	err_code = fds_write(BACKUP_FILE_ID, REC_KEY_LASTSEEN, testData, 3);
	APP_ERROR_CHECK(err_code);
	// Wait for write done event
	wait_for_fds_evt(FDS_EVT_WRITE);

	NRF_LOG_RAW_INFO("----Device Flash Init Done----\r\n\n\n");
#endif

	nrf_delay_ms(200);
	tstamp_reckey_init();

	// Start execution.
	timers_start();
	NRF_LOG_INFO("Timers Start.. \n");

	// Switch on advertising in SLOW connectable mode
	advMode = dynamic_advertising_handler(advMode, DYNADV_EVT_BUTTON_PRESS);
	bsp_board_leds_on();
		
    // Enter main loop.
    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
        	app_sched_execute();
        	power_manage();
        }
    }
}

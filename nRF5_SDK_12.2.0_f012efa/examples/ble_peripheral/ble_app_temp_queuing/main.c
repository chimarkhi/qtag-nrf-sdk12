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
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf_log_ctrl.h"
#include "nrf_temp.h"
#include "ble_srv_common.h"
#include "app_timer.h"
#include "SEGGER_RTT.h"
#include "nrf_delay.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_gap.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_nvic.h"
#include "bledb.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "time.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "nrf_drv_twi.h"
#include "sht31.h"
#include "battery_level.h"
#include "ble_tbs.h"
#define  NRF_LOG_MODULE_NAME 						"APP"
#include "nrf_log.h"
#include "nrf_peripherals.h"

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define TSTAMP_INTERVAL_IN_MS						1000
#define ADV_INTERVAL_IN_MS							2500
#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define ADV_INTERVAL				    				MSEC_TO_UNITS(ADV_INTERVAL_IN_MS, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define ADVDATA_UPDATE_INTERVAL					APP_TIMER_TICKS(ADV_INTERVAL_IN_MS, APP_TIMER_PRESCALER)
#define TSTAMP_INTERVAL									APP_TIMER_TICKS(TSTAMP_INTERVAL_IN_MS, APP_TIMER_PRESCALER)
#define LOGINTERVAL_ADVINTERVAL_RATIO		2																/** Logging interval = log_adv_ratio*adv_interval **/

#define APP_BEACON_INFO_LENGTH          0x02                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x00                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_COMPANY_IDENTIFIER          0x128B                            /**< Company identifier for TagBox */
#define APP_BEACON_UUID                 0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */
#define DEVICE_NAME											"XTC821"
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN        /**< UUID type for the Nordic UART Service (vendor specific). */
#define DATAPACKET_UUID									0xAB04

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)  	          /**< Minimum acceptable connection interval (7.5 milli seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(12.5, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (1 second). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    20                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF		                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0   		                              /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3
#define APP_TIMER_OP_QUEUE_SIZE         4		                                 /**< Size of timer operation queues. */

#define ADV_BUTTON											0

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static ble_tbs_t												m_tbs;																			/**< Structure to identify the TagBox Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static nrf_drv_twi_t 										twi = NRF_DRV_TWI_INSTANCE(0);


// Global variables
uint16_t nusRecKey;
uint16_t nusCurrentKey;
time_t tstamp_sec;
volatile bool initFlag = false;
volatile bool gcDone = false;
volatile bool writeFlag = false;
volatile bool isAdvertising = false;
volatile uint8_t syncType = SYNCTYPE_STRAIGHT;

static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
};


APP_TIMER_DEF(m_advdata_update_timer);
APP_TIMER_DEF(m_dataToDB_timer);
APP_TIMER_DEF(m_tstamp_timer);


/***@brief function for initializing nrf logging.
*/
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function to get internal temperature of IC. Used for calibration */
uint32_t temperature_data_get(void)
{
    int32_t temp;
    uint32_t err_code;

    err_code = sd_temp_get(&temp);
    APP_ERROR_CHECK(err_code);

    temp = temp*25; // -200 for Minew_Tag16 and 07; -425 for Minew_tag13;
    int8_t exponent = -2;
    return ((exponent & 0xFF) << 24) | (temp & 0x00FFFFFF);
}


static void advdata_update(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
		uint16_t 			temp, humid_batt;
	
    ble_advdata_service_data_t service_data[1];
			
	
		#ifdef NRF51
			uint32_t temp_humid 	= 47;
			uint8_t batt_level 		= 74;
		#else 
			uint32_t temp_humid 	= get_temp_humid(&twi);
			uint8_t batt_level 		= get_battery_level();
		#endif
	  
		#ifdef CALIBRATION
			uint16_t temp_internal = (uint16_t)temperature_data_get();
		#endif
	
		if (temp_humid == I2C_READ_ERROR)
		{
			return;
		}
		else
		{
			temp					= (uint16_t)(temp_humid>>16);
			#ifdef CALIBRATION  // Internal temperature sent instead of humidity+battery 
				humid_batt  = temp_internal;
			#else
				humid_batt	= temp_humid<<8|batt_level;
			#endif
		}
		
		uint16_t timeStamp[2] = {(uint16_t)tstamp_sec,(uint16_t)(tstamp_sec>>16)};
		uint16_t recKey 	 		= get_recKey();
	
		uint16_t dataPacket[2*WORDLEN_DATAPACKET] = {APP_COMPANY_IDENTIFIER, recKey,
													 timeStamp[0], timeStamp[1],
													 temp,  			 humid_batt};	
	
		service_data[0].service_uuid = DATAPACKET_UUID;
    service_data[0].data.size    = 4*WORDLEN_DATAPACKET;
    service_data[0].data.p_data  = (uint8_t *) &dataPacket;



    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

//    ble_advdata_manuf_data_t manuf_specific_data;
//    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
//    manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
//    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    advdata.include_appearance  		= false;
		advdata.name_type               = BLE_ADVDATA_SHORT_NAME;
		advdata.short_name_len					= 6;
//    advdata.p_manuf_specific_data 	= &manuf_specific_data;

    advdata.flags		         				= flags;
    advdata.service_data_count   		= 1;
    advdata.p_service_data_array 		= service_data;

    err_code = ble_advdata_set(&advdata, NULL);
    //NRF_LOG_DEBUG("adv set err: %d\r\n",err_code);
    APP_ERROR_CHECK(err_code);
		
}


void indicate_advertising(void)
{
		if (isAdvertising)
		{
			// Turn on LED to indicate advertising
			nrf_gpio_pin_write(19,1); // Minew S1 v1.0 green LED
			//nrf_gpio_pin_write(17,1); 	// Minew S1 v0.9 blue LED		
			//NRF_LOG_DEBUG("LED ON");
	
			nrf_delay_ms(5);
			// Turn off LED 
			nrf_gpio_pin_write(19,0); // Minew S1 v0.9 green LED
			//nrf_gpio_pin_write(17,0);		// Minew S1 v0.9 blue LED
			//NRF_LOG_DEBUG("LED OFF\r\n");
		}
}


void advdata_update_timer_timeout_handler(void * p_context)
{
	advdata_update();
	indicate_advertising();
}

void tstamp_timer_timeout_handler(void * p_context)
{
	 ++tstamp_sec;
//	NRF_LOG_DEBUG("Timestamp %d\r\n",tstamp_sec);
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
		uint16_t dataLen 	= sizeof(data);
		uint32_t err_code;
		err_code = fds_read(BACKUP_FILE_ID, REC_KEY_LASTSEEN, data, dataLen);
		if (err_code != NRF_SUCCESS) NRF_LOG_ERROR("FDS read error %d", err_code);
		
		if ((data[0] <=  (uint32_t)0xFFFF) 			& 
			  (data[0] >= (uint32_t)REC_KEY_START ) 	&
				err_code == FDS_SUCCESS	)
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


void dataToDB_timer_timeout_handler(void * p_context)
{
		time_t date_hour_seconds = tstamp_sec;
		uint16_t temp, humid_batt;
		
		
		uint32_t temp_humid		= get_temp_humid(&twi);
		uint8_t batt_level		= get_battery_level();

		#ifdef CALIBRATION
			uint16_t temp_internal = (uint16_t)temperature_data_get();
		#endif
	
		if (temp_humid == I2C_READ_ERROR)
		{	
			return;
		}
		else
		{
			temp				= (uint16_t)(temp_humid>>16);
			#ifdef CALIBRATION  // Internal temperature sent instead of humidity+battery 
				humid_batt  = temp_internal;
			#else
				humid_batt	= temp_humid<<8 | batt_level;
			#endif
		}
		
		uint32_t timeStamp = date_hour_seconds;
		uint32_t recKey = get_recKey();
		
		uint32_t dataPacket[WORDLEN_DATAPACKET] = {recKey,
														 timeStamp,
														 (temp << 16) | humid_batt};	
		NRF_LOG_RAW_INFO("\r\n\n\n");
		NRF_LOG_DEBUG("Data to DB: RecKey %08x, time %08x, data %08x\r\n", dataPacket[0], dataPacket[1], dataPacket[2]);
		uint32_t err_code = dataToDB(FILE_ID, recKey, dataPacket, WORDLEN_DATAPACKET);
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
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

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
    uint32_t err_code = app_timer_start(m_advdata_update_timer, ADVDATA_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_dataToDB_timer, LOGINTERVAL_ADVINTERVAL_RATIO*ADVDATA_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_tstamp_timer, TSTAMP_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}

static void gap_params_init(void)
{
		uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
		APP_ERROR_CHECK(err_code);

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
			uint16_t inRecKey = p_data[1]<<8|p_data[0];
//			uint16_t inRecKey = (((p_data[0]-0x30)&0x0F)<<12|
//													 ((p_data[1]-0x30)&0x0F)<<8|
//													 ((p_data[2]-0x30)&0x0F)<<4|
//													 ((p_data[3]-0x30)&0x0F));
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

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
		
	  // Add Nordic Uart Service to GATT 
    err_code = ble_nus_init(&m_nus, &nus_init);
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



/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;
		
    err_code = ble_advdata_set(&advdata, NULL);
		APP_ERROR_CHECK(err_code);
		
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

//		uint8_t adv_mult 				 = ble_tbs_get_advinterval(&m_tbs);
//		uint16_t adv_interval		 = MSEC_TO_UNITS(adv_mult*ADV_INTERVAL_UNIT_IN_MS, UNIT_0_625_MS);
		
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
		m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static uint32_t advertising_start(void)
{
    uint32_t err_code =  NRF_SUCCESS;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
		if (err_code == NRF_SUCCESS)
		{
			isAdvertising = true;
			NRF_LOG_INFO("Advertising started");
		}
		return err_code;
}

/**@brief Function for stopping advertising.
 */
static uint32_t advertising_stop(void)
{
    uint32_t err_code;
		if(isAdvertising)
		{
			err_code = sd_ble_gap_adv_stop();
		}
		else NRF_LOG_WARNING("/r/n/nAdv stop called, but already not advertising/r/n/n");
		
		if (err_code == NRF_SUCCESS) 
		{
			isAdvertising = false;
			NRF_LOG_INFO("Advertising stopped");	
		}
		return err_code;
}



/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
		uint8_t nus_status;
	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						isAdvertising = false;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						nrf_gpio_pin_write(18,1);				// Minew S1 v1.0 Red LED
						//nrf_gpio_pin_write(18,1);			// Minew S1 v0.9 Red LED	
						break; // BLE_GAP_EVT_CONNECTED

				case BLE_EVT_TX_COMPLETE:
            // Send next key event						
						NRF_LOG_INFO("\r\nTX Complete\r\n");
            //nus_tx_flag_set();
						nusRecKey++;
						nus_status = check_nusRecKey(nusRecKey, nusCurrentKey);
						if(nus_status == NUS_CONTINUE) payload_to_central_async(&m_nus, nusRecKey);
						else if (nus_status == NUS_STOP)
						{
							err_code = nus_eom_send(&m_nus, NUS_MSGTYPE_EOM);
								if (err_code != NRF_SUCCESS) { 
									NRF_LOG_ERROR("Err sending eom package: %d\r\n", err_code);
								}
								else NRF_LOG_INFO("EOM sent \r\n");
						}
						break; // BLE_EVT_TX_COMPLETE
				
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_write(18,0);				// Minew S1 v1.0 Red LED
            //nrf_gpio_pin_write(18,0);			// Minew S1 v0.9 Red LED
						syncType = SYNCTYPE_STRAIGHT;
						err_code = advertising_start();
						if (err_code !=NRF_SUCCESS) 
						{
							NRF_LOG_ERROR("Adv restart failed after disconnet errcode: %d\r\n",err_code);						
						}
						else  NRF_LOG_INFO("Adv restarted after disconnect\r\n\n"); 
						m_conn_handle = BLE_CONN_HANDLE_INVALID;
						break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

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

				case BLE_GATTS_EVT_WRITE:
				{
						uint8_t * pdata = p_ble_evt->evt.gatts_evt.params.write.data;
						ble_uuid_t uuid = p_ble_evt->evt.gatts_evt.params.write.uuid;
						SEGGER_RTT_printf(0, "\r\nChar value %d, uuid %04x\r\n", pdata[0], uuid.uuid);
					//ble_tbs_get_advinterval(&m_tbs);
				}
						break;
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
	 ble_tbs_on_ble_evt(&m_tbs, p_ble_evt);
//   ble_ias_on_ble_evt(&m_ias, p_ble_evt);
//   ble_lls_on_ble_evt(&m_lls, p_ble_evt);
//   ble_bas_on_ble_evt(&m_bas, p_ble_evt);
//   ble_ias_c_on_ble_evt(&m_ias_c, p_ble_evt);
//   ble_tps_on_ble_evt(&m_tps, p_ble_evt);
   bsp_btn_ble_on_ble_evt(p_ble_evt);
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
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;
				case BSP_EVENT_ADVERTISING_START:
						if (!isAdvertising) advertising_start();
						else advertising_stop();
						break;
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
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
		nrf_delay_ms(2000); // To allow for bounce during battery insertion 
		
		// Initialize.
		log_init();
		NRF_LOG_INFO("Starting Logging\n");
		NRF_LOG_INFO("\r\n\n\n\nInitializing ...\n");

		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
		err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
		APP_ERROR_CHECK(err_code);

		NRF_LOG_INFO("Initializing Timers \n");
		timers_init();
		NRF_LOG_INFO("Timers Initialized \n");

		err_code = buttons_leds_init(false);
		if (err_code != NRF_SUCCESS) 
		{
			NRF_LOG_ERROR("BSP init error %d\r\n", err_code); 
		}
		else NRF_LOG_INFO("BSP Initialized \n");
	
	
		ble_stack_init();	
		gap_params_init();
		NRF_LOG_INFO("ble stack Initialized \n");		
			
		adc_configure();
		advertising_init();
		NRF_LOG_INFO("Adv Initialized \n");
		
		err_code = services_init();
		if (err_code != NRF_SUCCESS) NRF_LOG_ERROR("Services init error %d\r\n", err_code); 
		//else NRF_LOG_INFO("Services Initialized \n");
		

		conn_params_init();
		NRF_LOG_INFO("Conn Params Initialized \n");
		
		err_code = twi_init(&twi);
		NRF_LOG_INFO("SHT31's I2C Initialized \n");

		// Wait for fds to be initialized before any read/write
		err_code =fds_bledb_init();
		if (err_code != NRF_SUCCESS) 
		{
			NRF_LOG_ERROR("fds init err: %d \n",err_code);
		}
		else NRF_LOG_INFO("fds initialized\r\n"); 
		APP_ERROR_CHECK(err_code);
		// POLL FOR INIT CALLBACK
		wait_for_fds_evt(FDS_EVT_INIT);
		
		
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

		#ifdef CALIBRATION
		nrf_delay_ms(1000);
		NRF_LOG_RAW_INFO("\r\n\n\n----THIS IS A CALIBRATION DEVICE ----\r\n\n\n");
		#endif
		
		nrf_delay_ms(1000);
		
		tstamp_reckey_init();		

		// Start execution.
		nrf_delay_ms(1000);
		timers_start();
		NRF_LOG_INFO("Times Start.. \n");

		err_code = advertising_start();
		NRF_LOG_INFO("adv start err: %d \n",err_code);
		APP_ERROR_CHECK(err_code);
		NRF_LOG_INFO("Started Advertising \n");
		
		bsp_board_leds_on();
		

    // Enter main loop.
    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
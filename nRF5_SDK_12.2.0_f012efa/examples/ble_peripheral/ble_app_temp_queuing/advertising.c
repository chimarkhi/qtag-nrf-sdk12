#define  NRF_LOG_MODULE_NAME 		"ADVERTISING"
#include "nrf_log.h"

#include "advertising.h"


static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
};



advertising_mode_t advertising_init()
{
    uint32_t      				err_code;
    ble_advdata_t 				advdata;
    ble_advdata_manuf_data_t 	manuf_specific_data;

    advertising_mode_t 			advMode = DYNADV_ADV_MODE_OFF;

    manuf_specific_data.company_identifier 	= APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data 		= (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   		= APP_BEACON_INFO_LENGTH;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = dynadv_flags[advMode];
    advdata.p_manuf_specific_data = &manuf_specific_data;
		
    err_code = ble_advdata_set(&advdata, NULL);
	APP_ERROR_CHECK(err_code);
		
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
	m_adv_params	  	 =  m_dynadv_params[advMode];

	return advMode;
}

void advdata_update(advertising_mode_t advMode)
{
    uint32_t      	err_code;
    ble_advdata_t 	advdata;
    uint8_t       	flags = dynadv_flags[advMode];
	uint16_t 		temp, recKey;
	uint8_t 		humid, batt_level;
	uint32_t 		tstamp_sec;
	
    ble_advdata_service_data_t service_data[1];
		
	err_code = get_telemetry_data(&temp, &humid, &batt_level, &tstamp_sec, &recKey);
	if (err_code !=NRF_SUCCESS) return; // Do not update adv data in case of errors

	uint16_t timeStamp[2] = {(uint16_t)tstamp_sec,(uint16_t)(tstamp_sec>>16)};

	uint16_t dataPacket[2*WORDLEN_DATAPACKET] = {APP_COMPANY_IDENTIFIER, recKey,
												 timeStamp[0], timeStamp[1],
												 temp,  			 (humid<<8)|batt_level};

	service_data[0].service_uuid = DATAPACKET_UUID;
    service_data[0].data.size    = 4*WORDLEN_DATAPACKET;
    service_data[0].data.p_data  = (uint8_t *) &dataPacket;


    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.include_appearance  		= false;
	advdata.name_type               	= BLE_ADVDATA_SHORT_NAME;
	advdata.short_name_len				= 6;

    advdata.flags	      				= flags;
    advdata.service_data_count   		= 1;
    advdata.p_service_data_array 		= service_data;

    err_code = ble_advdata_set(&advdata, NULL);
		
    if (err_code != NRF_SUCCESS){
		NRF_LOG_ERROR("Advertised Data update err: %d\r\n",err_code);
    }
    return;		
}

/**@brief 		Function for starting advertising.
	 @details   advertising started with given advertising_mode_t if currently device is not advertising 
	 @input 		advModeNew: mode in which advertising needs to be started
	 @inout 		advMode: current mode, updated to advModeNew if succesfully started advertising
*/
static uint32_t advertising_start(advertising_mode_t* p_advMode, advertising_mode_t advModeNew)
{
	uint32_t err_code	=  NRF_SUCCESS;

	m_adv_params = m_dynadv_params[advModeNew];

	if (*p_advMode == DYNADV_ADV_MODE_OFF) err_code = sd_ble_gap_adv_start(&m_adv_params);
	else return NRF_ERROR_INVALID_STATE;
	
	if (err_code == NRF_SUCCESS)
	{
		err_code = dynadv_timer_update(ADVDATA_UPDATE_INTERVAL*m_dynadv_params[advModeNew].interval/
										m_dynadv_params[DYNADV_ADV_MODE_FAST].interval);
		*p_advMode = advModeNew;
		NRF_LOG_DEBUG("Advertising Started in advMode: %d\r\n", advModeNew);
	}
	return err_code;
}

/**@brief Function for stopping advertising.
 */
static uint32_t advertising_stop(advertising_mode_t* p_advMode)
{
    uint32_t err_code;
	if(*p_advMode != DYNADV_ADV_MODE_OFF)
	{
		err_code = sd_ble_gap_adv_stop();
	}
	else
	{
		NRF_LOG_WARNING("Adv stop called, but already not advertising\r\n");
		return NRF_ERROR_INVALID_STATE;
	}
	if (err_code == NRF_SUCCESS)
	{
		*p_advMode = DYNADV_ADV_MODE_OFF;
		NRF_LOG_INFO("Advertising stopped\r\n");
	}
	return err_code;
}

static uint32_t advertising_update_advInterval(advertising_mode_t* p_advMode, advertising_mode_t advModeNew)
{
	uint32_t err_code;
	if (*p_advMode == advModeNew) return NRF_SUCCESS; // Nothing to be done

	err_code = advertising_stop(p_advMode);
	if (err_code == NRF_SUCCESS) {
		err_code = advertising_start(p_advMode, advModeNew);
	}
	else return err_code;

	if (err_code == NRF_SUCCESS) {
		*p_advMode = advModeNew;
		NRF_LOG_INFO("Advertising rate update to Mode : %d\r\n", advModeNew);
	}
	return err_code;
}


void indicate_advertising(advertising_mode_t advMode)
{
	if (advMode != DYNADV_ADV_MODE_OFF)
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

advertising_mode_t dynamic_advertising_handler(advertising_mode_t advMode, dynamic_advertising_event_t advEvent)
{
	uint32_t err_code;
	NRF_LOG_DEBUG("In Dyn Adv Handler, Mode:%d\r\n", advMode, advEvent);
	switch(advEvent)
	{
		case DYNADV_EVT_GATEWAY_FOUND:
			if (advMode == DYNADV_ADV_MODE_SLOW)	{ // Add GW whitelisting filter policy here
				err_code = advertising_update_advInterval(&advMode, DYNADV_ADV_MODE_FAST);
				if (err_code == NRF_SUCCESS) {
					NRF_LOG_INFO("Started Fast Advertising @ GW FOUND \r\n");
					dynadv_timeout_timer_start(DYNADV_FAST_MODE_TIMEOUT);	// Start a timer to timeout the Fast Adv Mode
				}
				else NRF_LOG_ERROR("Failed to execute FAST ADV @ GW FOUND, error:%d\r\n", err_code);
			}
			return advMode;

		case DYNADV_EVT_DATA_SYNCED:
			err_code = advertising_start(&advMode, DYNADV_ADV_MODE_SLOW_UNCONN);
			if (err_code == NRF_SUCCESS) {
				NRF_LOG_INFO("Adv changed to SLOW UN-CONN @ DATA SYNCED\r\n");
				dynadv_timeout_timer_start(DYNADV_SLOW_UNCONN_MODE_TIMEOUT);	// Start a timer to timeout the Unconnectable Adv Mode
			}
			else NRF_LOG_ERROR("Failed to move to SLOW UN-CONN @ DATA SYNCED, error:%d\r\n", err_code);
			return advMode;
				
		case DYNADV_EVT_CONN_DROPPED:
			err_code = advertising_start(&advMode, DYNADV_ADV_MODE_FAST);
			if (err_code == NRF_SUCCESS) {
				NRF_LOG_INFO("Connection dropped MID-SYNC, Restarted Advertising\r\n");
			}
			else NRF_LOG_ERROR("Connection dropped MID-SYNC, Error in restarting Advertising: %d\r\n", err_code);
			return advMode;

		case DYNADV_EVT_BUTTON_PRESS:
			if (advMode == DYNADV_ADV_MODE_OFF)
			{
				err_code = advertising_start(&advMode, DYNADV_ADV_MODE_SLOW);
				if (err_code == NRF_SUCCESS) {
					NRF_LOG_INFO("Started advertising on button-press in mode SLOW CONNECTABLE\r\n");
				}
				else NRF_LOG_ERROR("Failed to start Advertising at button-press, error:%d\r\n", err_code);
			}
			else {
				err_code = advertising_stop(&advMode);
				if (err_code == NRF_SUCCESS) {
					NRF_LOG_INFO("Stopped ADV @ BUTTON-PRESS \r\n");
				}
				else NRF_LOG_ERROR("Failed to stop ADV @ BUTTON-PRESS, error:%d\r\n", err_code);
			}
			return advMode;

		case DYNADV_EVT_FAST_MODE_TIMEOUT:
			if (advMode == DYNADV_ADV_MODE_FAST)	{
				err_code = advertising_update_advInterval(&advMode, DYNADV_ADV_MODE_SLOW);
				if (err_code == NRF_SUCCESS) {
					NRF_LOG_INFO("Changed to Slow Adv @ FAST ADV TIMEOUT\r\n");
				}
				else NRF_LOG_ERROR("Failed to execute Slow Adv @ FAST ADV TIMEOUT, error:%d\r\n", err_code);
			}
			return advMode;

		case DYNADV_EVT_UNCONN_MODE_TIMEOUT:
			if (advMode == DYNADV_ADV_MODE_SLOW_UNCONN)	{
				err_code = advertising_update_advInterval(&advMode, DYNADV_ADV_MODE_SLOW);
				if (err_code == NRF_SUCCESS) {
					NRF_LOG_INFO("Changed to Slow Adv @ SLOW UNCONN TIMEOUT\r\n");
				}
				else NRF_LOG_ERROR("Failed to execute Slow Adv @ SLOW UNCONN TIMEOUT, error:%d\r\n", err_code);
			}
			return advMode;

		case DYNADV_EVT_CONNECTED:
			if ((advMode == DYNADV_ADV_MODE_FAST) | (advMode == DYNADV_ADV_MODE_SLOW_UNCONN)){
				err_code = dynadv_timeout_timer_stop(); // TBD error handling
			}
			advMode = DYNADV_ADV_MODE_OFF;
			NRF_LOG_INFO("Connected to Client, Advertising stopped\r\n");
			return advMode;
		default:
			return advMode;
	}
}

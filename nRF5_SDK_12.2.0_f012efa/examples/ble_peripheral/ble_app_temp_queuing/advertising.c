#include "advertising.h"




void advertising_init(void)
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

void advdata_update(void)
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
	
		if (temp_humid == I2C_READ_ERROR)
		{
			return;
		}
		else
		{
			temp					= (uint16_t)(temp_humid>>16);
			humid_batt		= temp_humid<<8|batt_level;
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

    ble_advdata_manuf_data_t manuf_specific_data;
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    advdata.include_appearance  		= false;
		advdata.name_type               = BLE_ADVDATA_SHORT_NAME;
		advdata.short_name_len					= 6;
//    advdata.p_manuf_specific_data 	= &manuf_specific_data;

    advdata.flags		         				= flags;
    advdata.service_data_count   		= 1;
    advdata.p_service_data_array 		= service_data;

		// scan response packet
    ble_advdata_t   advdata_response;
    memset(&advdata_response, 0, sizeof(advdata_response));
    advdata_response.name_type               = BLE_ADVDATA_NO_NAME; 
    advdata_response.p_manuf_specific_data   = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, &advdata_response);
		
    if (err_code != NRF_SUCCESS){
				NRF_LOG_ERROR("adv set err: %d\r\n",err_code);
				nrf_delay_ms(500);	
    }
		APP_ERROR_CHECK(err_code);
		
}


/**@brief Function for starting advertising.
 */
uint32_t advertising_start(void)
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
uint32_t advertising_stop(void)
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
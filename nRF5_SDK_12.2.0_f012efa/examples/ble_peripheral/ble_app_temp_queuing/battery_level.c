#include "battery_level.h"

#ifdef ADC_PRESENT
#include "nrf_drv_adc.h"
#else
#include "nrf_drv_saadc.h"
#endif //ADC_PRESENT



#ifdef ADC_PRESENT
static nrf_adc_value_t adc_buf[1];
#else
static nrf_saadc_value_t adc_buf[2];
#endif //ADC_PRESENT


/** @brief Function for converting the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
static uint8_t battery_level_in_percent_fr03(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}

#ifdef ADC_PRESENT
/**@brief Function for handling the ADC driver eevent.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        nrf_adc_value_t adc_result;
        uint16_t        batt_lvl_in_milli_volts;
        uint8_t         percentage_batt_lvl;
        uint32_t        err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_adc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent_fr03(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl);
        if (
            (err_code != NRF_SUCCESS)
            &&
            (err_code != NRF_ERROR_INVALID_STATE)
            &&
            (err_code != BLE_ERROR_NO_TX_PACKETS)
            &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}


#else // SAADC_PRESENT
/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent_fr03(batt_lvl_in_milli_volts);
        
				if (err_code != NRF_SUCCESS)
				{
					SEGGER_RTT_printf(0,"Battery level conversion failed\r\n");
				}
				else
				{
					SEGGER_RTT_printf(0,"Battery level (from callback): %d\r\n",percentage_batt_lvl);
				}
			
    }
}


#endif // ADC_PRESENT


/**@brief Function for configuring ADC to do battery level conversion.
 */
void adc_configure(void)
{
    #ifdef ADC_PRESENT
    ret_code_t err_code = nrf_drv_adc_init(NULL, adc_event_handler);
    APP_ERROR_CHECK(err_code);

    static nrf_drv_adc_channel_t channel =
        NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_DISABLED);
    // channel.config.config.input = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    channel.config.config.input = (uint32_t)NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD;
    nrf_drv_adc_channel_enable(&channel);

    err_code = nrf_drv_adc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);
    #else //  SAADC_PRESENT
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
//   APP_ERROR_CHECK(err_code);
    #endif //ADC_PRESENT
}

uint8_t get_battery_level(void)
{
		nrf_saadc_value_t			batt_soc;
		uint8_t percentage_batt_lvl;
		uint32_t err_code;
	
    err_code = nrf_drv_saadc_sample_convert(0,&batt_soc);
	
		if (err_code != NRF_SUCCESS)
		{
			percentage_batt_lvl = ADC_CONVERT_ERROR;
			SEGGER_RTT_printf(0,"ADC single conv error : %d", err_code);
		}
		else
		{
			uint16_t batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(batt_soc) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
			percentage_batt_lvl = battery_level_in_percent_fr03(batt_lvl_in_milli_volts);
		}
		return percentage_batt_lvl;
}
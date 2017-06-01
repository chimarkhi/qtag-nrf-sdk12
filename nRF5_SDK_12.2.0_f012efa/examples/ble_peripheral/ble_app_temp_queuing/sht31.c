#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"
#include "sht31.h"


uint8_t device_address = 0; // Address used to temporarily store the current address being checked
uint8_t device_data = 0;
bool device_found = false; 

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   

    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            // If EVT_DONE (event done) is received a device is found and responding on that particular address
            SEGGER_RTT_printf(0,"!1*!\r 7-bit addr: %#x\r\n!*!\r\n\r\n", device_address);
            SEGGER_RTT_printf(0,"i-x:%d\r\n", device_data);
						device_found = true;
						break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            SEGGER_RTT_printf(0,"X%#x\r\n", device_address);
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            SEGGER_RTT_printf(0,"No data ACK: %#x\r\n", device_address);
            break;
        default:
            break;        
    }   
}


/**
 * @brief twi initialization.
 */
uint32_t twi_init (nrf_drv_twi_t * p_twi)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_config = {
       .scl                = DEVICE_SCL_PIN,
       .sda                = DEVICE_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    //err_code = nrf_drv_twi_init(p_twi, p_twi, twi_handler, NULL);
    err_code = nrf_drv_twi_init(p_twi, &twi_config, NULL, NULL);
    if (err_code != NRF_SUCCESS){
			SEGGER_RTT_printf(0,"Err in I2C init: %d",err_code);
			return err_code;
		}
	
		return NRF_SUCCESS;
}

/**
 * @brief Retrieve temp/humid data through i2c bus
 */
uint32_t get_temp_humid(nrf_drv_twi_t * p_twi)
	{
		nrf_drv_twi_enable(p_twi);
		
		uint8_t rx_data[6]={0};
		uint32_t err_code;
		uint8_t config_sht[] ={0x2C,0x06};

		
		err_code = nrf_drv_twi_tx(p_twi, SLAVE_ADDRESS, config_sht, sizeof(config_sht), true);
    if (err_code != NRF_SUCCESS){
			SEGGER_RTT_printf(0,"I2C tx err: %d\r\n",err_code);
			return err_code;
		}
			
		err_code = nrf_drv_twi_rx(p_twi, SLAVE_ADDRESS, rx_data, sizeof(rx_data));
		//APP_ERROR_CHECK(err_code);
    if (err_code != NRF_SUCCESS){
			SEGGER_RTT_printf(0,"I2C rx err: %d\r\n",err_code);
			return I2C_READ_ERROR;
		}
		
		nrf_drv_twi_disable(p_twi);
		
//		for (int8_t i=0; i<sizeof(rx_data);i++){
//		SEGGER_RTT_printf(0,"%d, ",rx_data[i]);
//		}
//		SEGGER_RTT_printf(0,"\r\n");
		
		int16_t temp = (((rx_data[0] * 256) + rx_data[1])) / 3.74  - 4500;
		uint16_t humid = (((rx_data[3] * 256) + rx_data[4])) / 655;
		
		return (temp<<16|humid);
}
	
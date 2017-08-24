#include <stdbool.h>
#include <stdint.h>
#include "sdk_errors.h"
#include "ble_advertising.h"
#include "main.h"


#define DYNADV_ADVRATE_MULTIPLE_SLOW				10			// Max adv rate is 10.24s
#define DYNADV_FAST_MODE_TIMEOUT					APP_TIMER_TICKS(60*1000, APP_TIMER_PRESCALER)
#define DYNADV_SLOW_UNCONN_MODE_TIMEOUT				APP_TIMER_TICKS(180*1000, APP_TIMER_PRESCALER)

/**@brief Advertising modes
 * @note 
 */
typedef enum
{
    DYNADV_ADV_MODE_OFF  = 0,                  			/** Non-advertising mode*/
    DYNADV_ADV_MODE_FAST,                    			/** FAST frequency advertising mode */
	DYNADV_ADV_MODE_SLOW,								/** Low frequency advertising mode */
	DYNADV_ADV_MODE_SLOW_UNCONN,                    	/** Low frequency non-connectable advertising mode */
	DYNADV_ADV_MODE_OFF_CONN,            	        	/** Low frequency non-connectable advertising mode */
	DYNADV_NUMBER_OF_MODES								/** Count of Modes */
} advertising_mode_t;


#define DYNADV_CH_MASK  {0,0,0}							// Make 1 to switch off the 3 advertising channels


/**< Adv Params for different advertising modes **/
static ble_gap_adv_params_t m_dynadv_params[DYNADV_NUMBER_OF_MODES] =
	{{BLE_GAP_ADV_TYPE_ADV_IND, NULL, BLE_GAP_ADV_FP_ANY, ADV_INTERVAL, APP_CFG_NON_CONN_ADV_TIMEOUT, DYNADV_CH_MASK}, 	//DYNADV_ADV_MODE_OFF
	 {BLE_GAP_ADV_TYPE_ADV_IND, NULL, BLE_GAP_ADV_FP_ANY, ADV_INTERVAL, APP_CFG_NON_CONN_ADV_TIMEOUT, DYNADV_CH_MASK}, 	//DYNADV_ADV_MODE_FAST
	 {BLE_GAP_ADV_TYPE_ADV_IND, NULL, BLE_GAP_ADV_FP_ANY, ADV_INTERVAL*DYNADV_ADVRATE_MULTIPLE_SLOW, APP_CFG_NON_CONN_ADV_TIMEOUT, DYNADV_CH_MASK}, 	//DYNADV_ADV_MODE_SLOW
	 {BLE_GAP_ADV_TYPE_ADV_NONCONN_IND, NULL, BLE_GAP_ADV_FP_ANY, ADV_INTERVAL*DYNADV_ADVRATE_MULTIPLE_SLOW, APP_CFG_NON_CONN_ADV_TIMEOUT, DYNADV_CH_MASK}, //DYNADV_ADV_MODE_SLOW_UNCONN
	 {BLE_GAP_ADV_TYPE_ADV_NONCONN_IND, NULL, BLE_GAP_ADV_FP_ANY, ADV_INTERVAL, APP_CFG_NON_CONN_ADV_TIMEOUT, DYNADV_CH_MASK}};	// OFF Connected

static uint8_t dynadv_flags[DYNADV_NUMBER_OF_MODES]	=
    {BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED,
     BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED,
	 BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED,
	 BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE,
	 BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED};	//unsued, since it is for OFF Connected mode


/**@brief Dynamic advertising scenarios
 * @note 
 */
typedef enum
{    
	DYNADV_EVT_GATEWAY_FOUND = 0,                  /**< Gateway detected in vicinity through Scan Response Request report*/
	DYNADV_EVT_DATA_SYNCED,
	DYNADV_EVT_CONN_DROPPED,
	DYNADV_EVT_BUTTON_PRESS,
	DYNADV_EVT_FAST_MODE_TIMEOUT,
	DYNADV_EVT_UNCONN_MODE_TIMEOUT,
	DYNADV_EVT_CONNECTED
} dynamic_advertising_event_t;


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
advertising_mode_t advertising_init(void);


/**@brief Function for updating advertising data, i.e. RECKEY, timestamp and telemetry data
 */
void advdata_update(advertising_mode_t advMode);


/**@brief Fuction to indicate advertising on LED
*/
void indicate_advertising(advertising_mode_t advMode);

/**@brief Dynamic advertising handler function. 
	 @details update the advertising mode based on a environment event	 
	 @input advMode : current advertising state of the device
	 @input advEvent:	detected ambient/device's event
*/
advertising_mode_t dynamic_advertising_handler(advertising_mode_t advMode, dynamic_advertising_event_t advEvent);


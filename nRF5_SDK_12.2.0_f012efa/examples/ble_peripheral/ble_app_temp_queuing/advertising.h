#include <stdbool.h>
#include <stdint.h>
#include "sdk_errors.h"
#include "ble_advertising.h"


/**@brief Adverting modes
 * @note 
 */
typedef enum
{
    ADV_MODE_OFF = 0,                  /**< Non-advertising mode*/
    ADV_MODE_SLOW,                      /**< Low frequency advertising mode */
    ADV_MODE_FAST                       /**< Low frequency advertising mode */
} advertising_mode_t;

/**@brief Dynamic advertising scenarios
 * @note 
 */
typedef enum
{    
		GATEWAY_IN_VICINITY = 0,                  /**< Non-advertising mode*/
    
} dynamic_adv_event_t;


extern volatile advertising_mode_t advMode;

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(void);

/**@brief Function for updating advertising data, i.e. RECKEY, timestamp and telemetry data
 */
void advdata_update(void);

/**@brief Function for starting advertising.
 */
uint32_t advertising_start(void);

/**@brief Function for stopping advertising.
 */
uint32_t advertising_stop(void);

/**@brief Fuction to indicate advertising on LED
*/
void indicate_advertising(void);

void adaptive_advertising(dynamic_adv_event_t);
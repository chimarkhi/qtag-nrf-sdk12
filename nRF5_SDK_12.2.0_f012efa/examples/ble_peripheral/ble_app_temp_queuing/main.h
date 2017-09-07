#include <stdbool.h>
#include <stdint.h>
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
#include "SEGGER_RTT.h"
#include "nrf_delay.h"
#include "ble_advdata.h"
#include "ble_gap.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_nvic.h"
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
//#include "ble_tbs.h"
#include "nrf_peripherals.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"



#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define TSTAMP_INTERVAL_IN_MS			1000
#define ADV_INTERVAL_IN_MS				2500
#define ADV_INTERVAL				    MSEC_TO_UNITS(ADV_INTERVAL_IN_MS, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define ADVDATA_UPDATE_INTERVAL			APP_TIMER_TICKS(ADV_INTERVAL_IN_MS, APP_TIMER_PRESCALER)
#define TSTAMP_INTERVAL					APP_TIMER_TICKS(TSTAMP_INTERVAL_IN_MS, APP_TIMER_PRESCALER)
#define LOGINTERVAL_ADVINTERVAL_RATIO	120								  /** Logging interval = log_adv_ratio*adv_interval **/

#define APP_BEACON_INFO_LENGTH          0x02                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x00                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_COMPANY_IDENTIFIER          0x128B                            /**< Company identifier for TagBox */
#define APP_BEACON_UUID                 0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */
#define DEVICE_NAME						"BBDUT1"
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN        /**< UUID type for the Nordic UART Service (vendor specific). */
#define DATAPACKET_UUID					0xAB04

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)  	          /**< Minimum acceptable connection interval (7.5 milli seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(12.5, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (1 second). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_TIMER_PRESCALER             0   		                              /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3
#define APP_TIMER_OP_QUEUE_SIZE         10		                                 /**< Size of timer operation queues. */

#define ADV_BUTTON						0
#define LED_PIN_ADV						19
#define LED_PIN_CONNECTION				18

#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVT_SIZE
#define SCHED_QUEUE_SIZE                20


uint32_t get_telemetry_data(uint16_t* temp, uint8_t* humid, uint8_t* batt_level, uint32_t* timeStamp, uint16_t* recKey);

uint32_t dynadv_timer_update(uint32_t timeoutTicks);

uint32_t dynadv_timer_start(uint32_t timeoutTicks);

uint32_t dynadv_timer_stop(void);

uint32_t get_timeStamp(void);

void sync_handler(void);

void bsp_event_handler(bsp_event_t event);

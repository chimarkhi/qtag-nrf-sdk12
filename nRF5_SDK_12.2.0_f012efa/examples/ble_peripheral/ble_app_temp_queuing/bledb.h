#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "fds.h"
#include "sdk_errors.h"
#include "ble_nus.h"

#define FILE_ID 						0x1111
#define BACKUP_FILE_ID					0x1112
#define REC_KEY_START     				0x0001																	// Starting Record Key
#define REC_KEY_MAX	     				REC_KEY_START + DATA_POINTS - 1 				// Maximum Record Key
#define REC_KEY_LASTSEEN   				0xFFFE																	// Record Key where copy of latest data-point is stored
#define REC_KEY_EOM						0xFFFF																	// Record Key with NUS end of message data
#define DATA_POINTS						8192																		// Max data points the device can store
#define FREQ_OF_RUN_GC					25																			// Run GC after every FREQ_OF_RUN_GC records are deleted
#define WORDLEN_DATAPACKET				3

#define BLEDB_ERROR_BASE				0

#define BLEDB_SUCCESS 					(BLEDB_ERROR_BASE +0)
#define BLEDB_ERROR_NO_RECORD_FOUND		(BLEDB_ERROR_BASE +1)

#define NUS_MSGTYPE_EOM					0
#define NUS_MSGTYPE_DIRTYRECKEY			1

#define NUS_NOACTION					0
#define NUS_CONTINUE					1
#define NUS_STOP						2

#ifdef CONCAT_NUS_DATA
#define CONCAT_RATIO					3					// Max number of data-points in a single record
#define CONCAT_DATA_LEN					(4+(2*CONCAT_RATIO)) 	// FDS data length in words
#define CONCAT_TIMESTAMP_ERROR_MARGIN	2					// Max absolute error in timestamps allowed between data-points within 1 record
#endif

/**@brief Type of reckey scroll while transferring data over nus
 * @note 
 */
typedef enum
{    
SYNCTYPE_STRAIGHT = 0,
SYNCTYPE_ROLLOVER,
SYNCTYPE_INVALID
} sync_type_t;


extern uint16_t nusRecKey;
extern volatile bool initFlag;
extern volatile bool gcDone;
extern volatile bool writeFlag;


void my_fds_evt_handler(fds_evt_t const * const p_fds_evt);

ret_code_t fds_write(uint16_t fileID, uint16_t recKey, uint32_t data[], uint16_t dataLen);

ret_code_t fds_read(uint16_t fileID, uint16_t recKey, uint32_t data[], uint8_t* dataLen);

ret_code_t fds_find_and_delete (uint16_t fileID, uint16_t recKey);
	
ret_code_t fds_bledb_init (void);

ret_code_t dataToDB (uint16_t fileID, uint16_t recKey, uint32_t *data, uint16_t dataLen);

ret_code_t payload_to_central_async (ble_nus_t * p_nus, uint16_t nusRecKey);
				
void nus_tx_flag_set(void);
		
uint16_t get_recKey(void);
	
void recCounter_init(uint16_t reckey_init);

void wait_for_fds_evt(fds_evt_id_t id);

ret_code_t save_lastseen(uint16_t fileID, uint16_t recKey);

sync_type_t check_startRecKey(uint16_t inRecKey, uint16_t currentRecKey);

uint8_t check_nusRecKey(uint16_t inRecKey, uint16_t currentRecKey, sync_type_t syncType);

ret_code_t nus_eom_send(ble_nus_t * p_nus, uint8_t eomType);

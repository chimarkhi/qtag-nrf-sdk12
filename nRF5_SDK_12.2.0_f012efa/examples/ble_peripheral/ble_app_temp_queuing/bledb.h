
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "fds.h"
#include "sdk_errors.h"
#include "ble_nus.h"
#include "main.h"

#define FILE_ID 							0x1111
#define REC_KEY_START     		0x2222				// Starting Record Key
#define REC_KEY_MAX	     			0xEEEE				// Maximum Record Key
#define REC_KEY_LASTSEEN   		0xFFFE				// Record Key where copy of latest data-point is stored
#define REC_KEY_EOM						0xFFFD				// Record Key with NUS end of message data
#define DATA_POINTS						8192					// Max data points the device can store
#define FREQ_OF_RUN_GC				25						// Run GC after every FREQ_OF_RUN_GC records are deleted
#define WORDLEN_DATAPACKET		3

#define BLEDB_ERROR_BASE			0

#define BLEDB_SUCCESS 							(BLEDB_ERROR_BASE +0)
#define BLEDB_ERROR_NO_RECORD_FOUND (BLEDB_ERROR_BASE +1)

extern uint16_t nusRecKey;
extern volatile bool initFlag;
extern volatile bool gcDone;
extern volatile bool writeFlag;

/**@brief   Data type fed to DB
 *
 * @note    
 */
typedef struct
{
    uint16_t temp	;
	  uint16_t humid ;
    uint32_t timeStamp ;
} db_data_t;

/**@brief Macro for converting db_data_t struct to a single concatenated data field
 * 				and returning pointer to byte array
 * @param[in] uint16_t temp temp value from SHT20
 * @param[in] uint16_t humid humid value from SHT20	
 * @param[in] uint32_t timeStamp timer tick
* 	@param[out] uint8_t p_dataPacket : pointer to byte array containing concatenated data 
 */
#define CREATE_DATA_PACKET(temp, humid, timeStamp, p_dataPacket)                           \
    do                                                      \
    {                                                       \
        const uint64_t dataPacket = 											  \
					((uint64_t)timeStamp << 32)| 											\
				((uint32_t)temp <<16) | humid;    						      \
				uint8_t *p_dataPacket = (uint8_t *)&dataPacket;			\
    } while (0)

void my_fds_evt_handler(fds_evt_t const * const p_fds_evt);

ret_code_t fds_write(uint16_t fileID, uint16_t recKey, uint32_t data[], uint16_t dataLen);

ret_code_t fds_read(uint16_t fileID, uint16_t recKey, uint32_t data[], uint8_t dataLen);

ret_code_t fds_find_and_delete (uint16_t fileID, uint16_t recKey);
	
ret_code_t fds_bledb_init (void);

ret_code_t dataToDB (uint16_t fileID, uint16_t recKey, uint32_t *data, uint16_t dataLen);

static void create_nus_payload(uint32_t data, uint8_t dataByteArray, uint16_t dataLengthInBytes);
		
ret_code_t payload_to_central (ble_nus_t * p_nus, uint16_t startRecKey);

ret_code_t payload_to_central_async (ble_nus_t * p_nus, uint16_t nusRecKey);
				
void nus_tx_flag_set(void);
		
uint16_t get_recKey(void);
	
void recCounter_init(uint16_t reckey_init);

void wait_for_fds_evt(fds_evt_id_t id);

ret_code_t save_lastseen(uint16_t fileID, uint16_t recKey);

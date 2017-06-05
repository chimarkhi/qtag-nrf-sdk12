#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "nrf.h"
#include "fds.h"
#include "sdk_errors.h"
#include "SEGGER_RTT.h"
#include "bledb.h"
#include "app_util.h"
#include "nrf_delay.h"
#include "ble_nus.h"

volatile uint8_t nus_tx_complete = 0;

uint32_t recCounter;

void nus_tx_flag_set(void)
{
	nus_tx_complete = 1;
}

uint16_t get_recKey(void)
{
	return recCounter+REC_KEY_START;
}

void recCounter_init(uint16_t reckey_init)
{
	recCounter = reckey_init - REC_KEY_START;
}


void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
		uint32_t data[] = {0,0,0};
		uint8_t dataLen;
		uint16_t fileID = p_fds_evt->write.file_id;
		uint16_t recKey = p_fds_evt->write.record_key;
		uint16_t recID = p_fds_evt->write.record_id;

		
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != FDS_SUCCESS)
            {
              SEGGER_RTT_printf(0,"FDS Initialization Failed \n");  // Initialization failed.
            } 
						else initFlag = 1;
            break;
				case FDS_EVT_WRITE:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							writeFlag = 1;
							SEGGER_RTT_printf(0,"Wrote file,rec,recID :%04x,%04x,%d  recCounter: %d \r\n", 
																fileID, recKey, recID, recCounter);
							fds_read(fileID, recKey, data, dataLen);

							if (p_fds_evt->write.record_key != REC_KEY_LASTSEEN)
							{
								++recCounter;
							}
						}
						else
						{
							SEGGER_RTT_printf(0,"FDS write failed for file,record: %04x,%04x", fileID, recKey);
						}
						break;
				case FDS_EVT_UPDATE:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							SEGGER_RTT_printf(0,"Updated file,record,recID: %04x,%04x,%d \r\n", fileID, recKey, recID);
							fds_read(fileID, recKey, data, dataLen);
						}
						break;						
				case FDS_EVT_DEL_FILE:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							SEGGER_RTT_printf(0,"File Deleted and GC being run\n");
							fds_gc();
						}
						break;		
				case FDS_EVT_GC:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							SEGGER_RTT_printf(0,"GC Done\n");
							gcDone = 1;
						}
						break;		
        default:
            break;
    }
}



ret_code_t fds_write(uint16_t fileID, uint16_t recKey, uint32_t data[], uint16_t dataLen)
{
		fds_record_t        record;
		fds_record_desc_t   record_desc;
		fds_record_chunk_t  record_chunk;

		// Set up data.
		static uint32_t dataPacket_test[3];
		for(uint8_t i = 0; i < dataLen; i++) {
		dataPacket_test[i] = data[i];
		}
		
		record_chunk.p_data         = &dataPacket_test[0];
		record_chunk.length_words   = dataLen;
		// SEGGER_RTT_printf(0,"Write dataIn: %08x, %08x, %08x, Size:%d\r\n", data[0],data[1],data[2],record_chunk.length_words);
		// Set up record.
		record.file_id              = fileID;
		record.key              	  = recKey;
		record.data.p_chunks        = &record_chunk;
		record.data.num_chunks      = 1;
				
		ret_code_t ret = fds_record_write(&record_desc, &record);

		if (ret != FDS_SUCCESS)
		{
				return ret;
		}		
		SEGGER_RTT_printf(0,"Wrote recordID:%d \r\n",record_desc.record_id);
		return NRF_SUCCESS;
}


ret_code_t fds_update(uint16_t fileID, uint16_t recKey, uint32_t data[], uint16_t dataLen)
{
		fds_record_t        record;
		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_record_chunk_t  record_chunk;

		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint32_t err_code;
		uint32_t *dataTemp;

		// Set up data.
		static uint32_t dataPacket_test[3];
		for(uint8_t i = 0; i < dataLen; i++) {
		dataPacket_test[i] = data[i];
		}
		
		record_chunk.p_data         = &dataPacket_test[0];
		record_chunk.length_words   = dataLen;
		//SEGGER_RTT_printf(0,"Write dataIn: %08x, %08x, %08x, Size:%d\r\n", data[0],data[1],data[2],record_chunk.length_words);
		
		// Set up record.
		record.file_id              = fileID;
		record.key              	  = recKey;
		record.data.p_chunks        = &record_chunk;
		record.data.num_chunks      = 1;
		
		//SEGGER_RTT_printf(0,"Updating File/Rec : %04x/%04x",fileID, recKey);
		// Loop until all records with the given key and file ID have been found.
		while (fds_record_find(fileID, recKey, &record_desc, &ftok) == FDS_SUCCESS)
		{
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != FDS_SUCCESS)
				{
					SEGGER_RTT_printf(0,"error opening record: %u\n", err_code);
					return err_code;		
				}
				
				err_code = fds_record_update(&record_desc,&record);
				if (err_code != FDS_SUCCESS)
				{
					SEGGER_RTT_printf(0,"Update error : %d",err_code);
					return err_code;	
				}
		}				
		return NRF_SUCCESS;
}


ret_code_t fds_read(uint16_t fileID, uint16_t recKey, uint32_t data[], uint8_t dataLen)
{
		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint32_t err_code;
		uint32_t *dataTemp;
//		recKey = get_recKey();
		
		//SEGGER_RTT_printf(0,"Search for FILEID: %04x, RECKEY: %04x \r\n",fileID, recKey);
		// Loop until all records with the given key and file ID have been found.
		while (fds_record_find(fileID, recKey, &record_desc, &ftok) == FDS_SUCCESS)
		{
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != FDS_SUCCESS)
				{
					SEGGER_RTT_printf(0,"error opening record: %u\n", err_code);
					return err_code;		
				}
				
				SEGGER_RTT_printf(0,"Data read back from file,rec,recID [%04x,%04x,%d] = ",
														fileID, recKey, record_desc.record_id);
				dataTemp = (uint32_t *) flash_record.p_data;
				for (uint8_t i=0;i<flash_record.p_header->tl.length_words;i++)
				{
					SEGGER_RTT_printf(0,"%08x ",dataTemp[i]);
					data[i] = dataTemp[i];
				}
				dataLen = flash_record.p_header->tl.length_words;
				SEGGER_RTT_printf(0,"\r\n");
				// Access the record through the flash_record structure.
				// Close the record when done.
				err_code = fds_record_close(&record_desc);
				if (err_code != FDS_SUCCESS)
				{
					return err_code;	
				}
		}
		return NRF_SUCCESS;
}


ret_code_t fds_find_and_delete (uint16_t fileID, uint16_t recKey)
{
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
	
		ftok.page=0;
		ftok.p_addr=NULL;
		// Loop and find records with same ID and rec key and mark them as deleted. 
		while (fds_record_find(fileID, recKey, &record_desc, &ftok) == FDS_SUCCESS)
		{
			fds_record_delete(&record_desc);
			SEGGER_RTT_printf(0,"Deleted record ID: %d \r\n",record_desc.record_id);
		}
		// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret_code_t ret = fds_gc();
		if (ret != FDS_SUCCESS)
		{
			
			return ret;
		}
		return NRF_SUCCESS;
}


ret_code_t fds_bledb_init (void)
{
		recCounter = 0;
		ret_code_t ret = fds_register(my_fds_evt_handler);
		if (ret != FDS_SUCCESS)
		{
					return ret;
				
		}
		ret = fds_init();
		if (ret != FDS_SUCCESS)
		{
				return ret;
		}
		
		return NRF_SUCCESS;
		
}

ret_code_t fds_cleanup(uint32_t fileID)
{
	ret_code_t ret = fds_file_delete(fileID);
		if (ret == FDS_SUCCESS)
		{
			ret = fds_gc();
		}
		else
		{
			SEGGER_RTT_printf(0,"Flash cleanup error:%d",ret);
		}
	return ret;
}


ret_code_t dataToDB (uint16_t fileID, uint16_t recKey, uint32_t * data, uint16_t dataLen)
{
		recKey = REC_KEY_START + recCounter;
		//SEGGER_RTT_printf(0,"dataToDB dataArray : %08x %08x %08x\r\n", data[0], data[1], data[2]);
		ret_code_t ret = fds_write(fileID, recKey, data, dataLen);		

		//Handle exceptions like flash full etc : FDS_ERR_*	
		if (ret != FDS_SUCCESS)
		{
			switch (ret)
			{
				case FDS_ERR_OPERATION_TIMEOUT:
				case FDS_ERR_NO_SPACE_IN_FLASH:
					SEGGER_RTT_printf(0,"No space in flash, running GC\r\n");
					ret = fds_cleanup(fileID);
					//fds_gc();
					//while(!gcDone);
					//gcDone = false;
					//ret = fds_write(fileID, recKey, data, dataLen);
					return ret;
					break;
				case FDS_ERR_NO_PAGES:
				case FDS_ERR_BUSY:
				case FDS_ERR_INTERNAL:
				default:
					ret = FDS_ERR_INTERNAL;
          break;
			}
		}
		
		return NRF_SUCCESS;	
		
}

static void create_nus_payload(uint32_t data, uint8_t dataByteArray, uint16_t dataLengthInBytes)
{
			dataLengthInBytes = sizeof(data)<<2;
			uint8_t *p_dataPacket = (uint8_t *)&data;
			
			uint8_t temp[dataLengthInBytes];
			for (uint8_t i=0;i<sizeof(p_dataPacket);i++){	
				temp[i] = p_dataPacket[i];
			}
}


ret_code_t payload_to_central_async (ble_nus_t * p_nus, uint16_t nusRecKey)
{
		uint32_t recKey ;
		uint32_t data[] = {0,0,0};
		uint16_t dataLen;
		ret_code_t ret;
		uint16_t recKey_current = get_recKey();
		recKey = nusRecKey;
		
		if (recKey < recKey_current)
		{		
			ret = fds_read(FILE_ID, recKey, data, dataLen);
			//SEGGER_RTT_printf(0,"read err %d\r\n", ret);
			
			//	Handle exceptions like flash full etc : FDS_ERR_*	
			if (ret != FDS_SUCCESS)
			{
				SEGGER_RTT_printf(0,"Record read error: %d", ret);
				nrf_delay_ms(1000);
				switch (ret)
				{
					case FDS_ERR_OPERATION_TIMEOUT:
					case FDS_ERR_RECORD_TOO_LARGE:
					case FDS_ERR_NO_SPACE_IN_FLASH:
						SEGGER_RTT_printf(0,"No space in flash");
					case FDS_ERR_NO_PAGES:
					case FDS_ERR_BUSY:
					case FDS_ERR_INTERNAL:
					default:
						ret = FDS_ERR_INTERNAL;
						break;
				}
			}
			else {
				uint8_t *p_dataPacket = (uint8_t *)data;
				//uint8_t dataLengthInBytes = dataLen*4;
				uint8_t dataLengthInBytes = WORDLEN_DATAPACKET*4;
				uint8_t dataByteArray[dataLengthInBytes];
				for (uint8_t i=0;i<dataLengthInBytes;i++){
					dataByteArray[i] = p_dataPacket[i];
					//SEGGER_RTT_printf(0,"%02x",dataByteArray[i]);
				}

				ret = ble_nus_string_send(p_nus, p_dataPacket, dataLengthInBytes);
				if (ret != FDS_SUCCESS)
				{
					SEGGER_RTT_printf(0,"NUS string send error: %d",ret);
					return ret;
				}
					
				SEGGER_RTT_printf(0,"Data sent over NUS:");
				for (uint8_t i=0;i<dataLengthInBytes;i++){
					SEGGER_RTT_printf(0,"%02x",dataByteArray[i]);
				}
			}
			}
		return NRF_SUCCESS;
}

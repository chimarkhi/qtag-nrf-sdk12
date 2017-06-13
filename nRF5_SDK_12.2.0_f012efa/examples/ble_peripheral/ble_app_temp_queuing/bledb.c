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

		//SEGGER_RTT_printf(0,"FDS Evt %d, result %d\r\n",p_fds_evt->id ,p_fds_evt->result );
		
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != FDS_SUCCESS)
            {
              SEGGER_RTT_printf(0,"FDS Initialization Failed \n");  // Initialization failed.
            } 
						else initFlag = true;
            break;
				case FDS_EVT_WRITE:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							writeFlag = true;
							SEGGER_RTT_printf(0,"Wrote file,rec,recID :%04x,%04x,%d  recCounter: %d \r\n", 
																fileID, recKey, recID, recCounter);
							
							// if data saved successfully, update the last seen reckey and tstamp in flash
							if ((recKey != REC_KEY_LASTSEEN) & (recKey != REC_KEY_EOM)) 
							{
								fds_read(fileID, recKey, data, dataLen);
								++recCounter;
							}
							else fds_read(fileID, recKey, data, dataLen);
						}
						else
						{
							SEGGER_RTT_printf(0,"FDS write failed for file,record: %04x,%04x result:%d\r\n", fileID, recKey,p_fds_evt->result);
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
							fds_gc();
						}
						break;		
				case FDS_EVT_DEL_RECORD:
						SEGGER_RTT_printf(0,"Deleted record ID: %d, result: %d \r\n",p_fds_evt->del.record_id, p_fds_evt->result);
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							if ((p_fds_evt->del.record_id % FREQ_OF_RUN_GC < 2) & 
									(p_fds_evt->del.record_id != REC_KEY_LASTSEEN ))
							{
								SEGGER_RTT_printf(0,"Scheduled GC being run\r\n");
								fds_gc();
								//wait_for_fds_evt(FDS_EVT_GC);
							}
						}
				case FDS_EVT_GC:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							SEGGER_RTT_printf(0,"GC Done, space reclaimed %d\n", p_fds_evt->gc.space_reclaimed);
							gcDone = true;
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
//		SEGGER_RTT_printf(0,"Wrote recordID:%d \r\n",record_desc.record_id);
		return NRF_SUCCESS;
}


ret_code_t save_lastseen(uint16_t fileID, uint16_t recKey)
{
		fds_record_t        record;
		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_record_chunk_t  record_chunk;
		uint32_t 						data[3] = {0,0,0};
		
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint32_t err_code;
		uint32_t *dataTemp;

		
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
				SEGGER_RTT_printf(0,"\r\n");
				
				err_code = fds_write(FILE_ID, REC_KEY_LASTSEEN, dataTemp, flash_record.p_header->tl.length_words);
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
		uint16_t 						count = 0;
		ftok.page=0;
		ftok.p_addr=NULL;
		
		uint32_t err_code;
	
		// Loop and find records with same ID and rec key and mark them as deleted. 
		while (fds_record_find(fileID, recKey, &record_desc, &ftok) == FDS_SUCCESS)
		{
			err_code = fds_record_delete(&record_desc);
			//SEGGER_RTT_printf(0,"Deleted record ID: %d, err: %d \r\n",record_desc.record_id, err_code);
			count++;
		}
		if (count>0)	return NRF_SUCCESS;
		else return BLEDB_ERROR_NO_RECORD_FOUND;
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
		recKey = (recCounter % DATA_POINTS) + REC_KEY_START;
		
		//SEGGER_RTT_printf(0,"dataToDB dataArray : %08x %08x %08x\r\n", data[0], data[1], data[2]);
		ret_code_t ret = fds_find_and_delete(fileID, recKey);
		if (ret == FDS_SUCCESS)
		{
			SEGGER_RTT_printf(0,"Old record will be upated  \r\n");
		}
		else if (ret == FDS_ERR_OPERATION_TIMEOUT)
		{
			SEGGER_RTT_printf(0,"New record will be written \r\n");
		}
		else return ret;
			
		ret = fds_write(fileID, recKey, data, dataLen);		

		//Handle exceptions like flash full etc : FDS_ERR_*	
		if (ret != FDS_SUCCESS)
		{
			switch (ret)
			{
				case FDS_ERR_NO_SPACE_IN_FLASH:
					SEGGER_RTT_printf(0,"No space in flash, running GC\r\n");
					//ret = fds_cleanup(fileID);
					fds_gc();
					wait_for_fds_evt(FDS_EVT_GC);
					ret = fds_write(fileID, recKey, data, dataLen);
					wait_for_fds_evt(FDS_EVT_WRITE);
					return ret;
					break;
				default:
					return ret;
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

		// Map inbound nusRecKey to a record in the DB		
		if (nusRecKey != REC_KEY_EOM)  		
		{	
			recKey = ((nusRecKey-REC_KEY_START) % DATA_POINTS) + REC_KEY_START;
		}
		else
		{
			recKey = nusRecKey;
		}
		
		if ((recKey < recKey_current) | (recKey == REC_KEY_EOM)) 
		{		
			ret = fds_read(FILE_ID, recKey, data, dataLen);
			
			//	Handle exceptions like flash full etc : FDS_ERR_*	
			if (ret != FDS_SUCCESS)
			{
				SEGGER_RTT_printf(0,"Record read error: %d", ret);
				switch (ret)
				{
					case FDS_ERR_OPERATION_TIMEOUT:
						SEGGER_RTT_printf(0,"RECKEY recKey not found in DB\r\n", ret);
						return ret;
					default:
						ret = FDS_ERR_INTERNAL;
						break;
				}
			}
			else 
			{
				uint8_t *p_dataPacket = (uint8_t *)data;
				//uint8_t dataLengthInBytes = dataLen*4;
				uint8_t dataLengthInBytes = WORDLEN_DATAPACKET*4;
				uint8_t dataByteArray[dataLengthInBytes];
				for (uint8_t i=0;i<dataLengthInBytes;i++)
				{
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
				for (uint8_t i=0;i<dataLengthInBytes;i++)
				{
					SEGGER_RTT_printf(0,"%02x",dataByteArray[i]);
				}
			}
		}
		return NRF_SUCCESS;
}

void wait_for_fds_evt(fds_evt_id_t id)
{
	switch (id)
    {
        case FDS_EVT_INIT:
            while(!initFlag) __WFE;
						initFlag = false;
            break;

        case FDS_EVT_WRITE:
            while(!writeFlag) __WFE;
						initFlag = false;
            break;

				case FDS_EVT_GC:
            while(!gcDone) __WFE;
						initFlag = false;
            break;
						
        default:
            break;				
		}
}

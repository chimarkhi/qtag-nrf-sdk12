Advertised Packet : 

<Data>      <len>    <type>    <value>
Flags      :    2     0x01       0x04
SerData    :   15     0x16      Ser Data
Name       :    7     0x08      TagName(6 bytes)

Service Data : 
ServiceUUID (AB04)
Company Id  (128B)
uint16_t dataPacket = {APP_COMPANY_IDENTIFIER, recKey,
                        timeStamp[0], timeStamp[1],
			             temp,  			 humid}
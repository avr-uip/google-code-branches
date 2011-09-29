//  Vladimir S. Fonov ~ vladimir.fonov <at> gmail.com
// based on original code by 
//							edi87 [at] fibertel.com.ar
//								Jonathan Granade

#include "global-conf.h"
#include "enc28j60.h"
#include <avr/io.h>
//#include <util/delay.h>
//#include "network.h"

#define IP_TCP_HEADER_LENGTH 40
#define TOTAL_HEADER_LENGTH (IP_TCP_HEADER_LENGTH+ETHERNET_HEADER_LENGTH)

/*
void network_init(uint8_t *eth_addr)
{
	enc28j60Init(eth_addr,ENC28J60_ENABLE_DUPLEX);
}
*/
void network_init()
{
	enc28j60Init(ENC28J60_ENABLE_DUPLEX);
}

/*
void network_send(void){
	if(uip_len <= UIP_LLH_LEN + 40){
		enc28j60PacketSend(uip_len, (uint8_t *)uip_buf, 0, 0);
	}else{
		enc28j60PacketSend(54, (uint8_t *)uip_buf , uip_len - UIP_LLH_LEN - 40, (uint8_t*)uip_appdata);
	}
}
*/

void network_send(void)
{
	enc28j60PacketSend((uint8_t *)uip_buf, uip_len);
}

uint16_t network_read(void)
{
	uint16_t packetLength;
	
	packetLength = enc28j60BeginPacketReceive();

	// if there's no packet or an error - exit without ending the operation
	if( !packetLength )
	  return 0;

	// drop anything too big for the buffer
	if( packetLength > UIP_BUFSIZE )
	{
		enc28j60EndPacketReceive();
		return 0;
	}
	
	// copy the packet data into the uIP packet buffer
	enc28j60PacketReceive( uip_buf, packetLength );
	enc28j60EndPacketReceive();
		
	return packetLength;
}


uint8_t network_sending(void)
{
	return enc28j60PollPacketSending();
}

void network_get_MAC(uint8_t* macaddr)
{
	// read MAC address registers
	// NOTE: MAC address in ENC28J60 is byte-backward
	*macaddr++ = enc28j60Read(MAADR5);
	*macaddr++ = enc28j60Read(MAADR4);
	*macaddr++ = enc28j60Read(MAADR3);
	*macaddr++ = enc28j60Read(MAADR2);
	*macaddr++ = enc28j60Read(MAADR1);
	*macaddr++ = enc28j60Read(MAADR0);
}

void network_set_MAC(uint8_t* macaddr)
{
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR5, *macaddr++);
	enc28j60Write(MAADR4, *macaddr++);
	enc28j60Write(MAADR3, *macaddr++);
	enc28j60Write(MAADR2, *macaddr++);
	enc28j60Write(MAADR1, *macaddr++);
	enc28j60Write(MAADR0, *macaddr++);
}


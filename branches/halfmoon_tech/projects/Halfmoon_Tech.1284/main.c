#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "global-conf.h"
#include "timer.h"
#include <util/delay.h>

//#include "a2d.h"
#include "uart-t.h"

#include "eeprom_addr.h"

#include "uip_arp.h"
#include "network.h"
#include "enc28j60.h"
#include "apps-conf.h"
#include <string.h>

#include "net_conf.h"

#define NUM_TEMP_SENSORS 8
#define NUM_TEMP_READINGS 16


// this may need to be a 32 bit number instead
uint16_t temp_sensors[NUM_TEMP_SENSORS];

uint8_t serial_number[serial_len];
uint8_t mac_address[mac_len];

uint8_t dhcp_enabled[1];

uint8_t ip_address[ip_address_len];
uint8_t nm_address[nm_address_len];
uint8_t gw_address[gw_address_len];
uint8_t dns_address[dns_address_len];

void load_config(void){
	eeprom_read_block ((void *)serial_number, (const void *)serial_start,serial_len);
/*	eeprom_read_block ((void *)mac_address, (const void *)mac_start,mac_len);
	eeprom_read_block ((void *)dhcp_enabled, (const void *)dhcp_enabled_start,1);
	eeprom_read_block ((void *)ip_address, (const void *)ip_address_start,ip_address_len);
	eeprom_read_block ((void *)nm_address, (const void *)nm_address_start,nm_address_len);
	eeprom_read_block ((void *)gw_address, (const void *)gw_address_start,gw_address_len);
*/
// I still need to add the dns support
	eeprom_read_block ((void *)dns_address, (const void *)dns_address_start,dns_address_len);

}


/*
void read_sensors(void){
	extern int sensor0, sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7;

	int ii, iii, sample,temp;
	for(iii=0;iii<8;iii++){
		sample=0;
		for(ii=0;ii<16;ii++){
			//sample += a2dConvert10bit(iii);
		}
		temp = (sample / 16);
		//temp = convert_adc2far(temp);
		if(iii==0){sensor0 = temp;}
		else if(iii==1){sensor1 = temp;}
		else if(iii==2){sensor2 = temp;}
		else if(iii==3){sensor3 = temp;}
		else if(iii==4){sensor4 = temp;}
		else if(iii==5){sensor5 = temp;}
		else if(iii==6){sensor6 = temp;}
		else if(iii==7){sensor7 = temp;}
	}

int convert_adc2far(int adc_data){
    float ttt;
    ttt = (adc_data * .0048828125 * 100);
    ttt = ttt * 1.8;
   return (ttt + 32);
}

void read_sensors(void)
{
    uint8_t sensor_index, read_number;
    uint16_t sample;
    for(sensor_index = 0; sensor_index < NUM_TEMP_SENSORS; sensor_index++)
    {
        sample=0;
        for(read_number = 0; read_number < NUM_TEMP_READINGS; read_number++)
        {
            //sample += a2dConvert10bit(sensor_number);
        }
        sample = (sample / NUM_TEMP_READINGS);
        sample = convert_adc2far(sample);
        temp_sensors[sensor_index] = sample;
    }
}
*/

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
struct timer dhcp_timer;

int bob = 986;

int main(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

	load_config();

//led_conf();

    CLKPR=(1<<CLKPCE); // change enable
    CLKPR=0; // "no pre-scaler"
    _delay_loop_1(0); // 60us

    int i;
    struct timer periodic_timer, arp_timer, sensor_timer;

    clock_init();

    timer_set(&periodic_timer, CLOCK_SECOND);
    timer_set(&arp_timer, CLOCK_SECOND * 10);
    timer_set(&sensor_timer, CLOCK_SECOND * 5);

    uip_init();

    // must be done or sometimes arp doesn't work
    uip_arp_init();
    
    // load the network configs from eeprom
    net_conf_init();

    network_init_mac(net_conf_get_mac());

    // init temp array
    for( i = 0; i < NUM_TEMP_SENSORS; i++)
    {
        temp_sensors[i] = 0;
    }

    if (net_conf_is_dhcpc())
    {
        // setup the dhcp renew timer the make the first request
        timer_set(&dhcp_timer, CLOCK_SECOND * 600); // 10 minutes till renew
        dhcpc_init(net_conf_get_mac(), 6);
        dhcpc_request();
    }

    // start hosted services
    telnetd_init();
    httpd_init();
	//a2dInit();
	USART_Init(95);
	sendString("\E[H\E[J");
	sendString("Booting Biomass Ethernet\r\n");

	char bb[100];
	sprintf(bb, "Serial Number: %d%d%d%d%d%d%d%d%d%d\r\n", serial_number[0],serial_number[1],serial_number[2],serial_number[3],serial_number[4],serial_number[5],serial_number[6],serial_number[7],serial_number[8],serial_number[9]); 
	sendString(bb);
	memset(bb, 0,  sizeof(bb));
        sendString("MAC Address: ");
        net_conf_get_mac_string (bb, 100);
	sendString(bb);
	memset(bb, 0,  sizeof(bb));
	sprintf(bb, "USE DHCP: %d\r\n", net_conf_is_dhcpc()); 
	sendString(bb);
	memset(bb, 0,  sizeof(bb));
	sprintf(bb, "IP  Address: "); 
        net_conf_get_ip_string(bb, 100);
	sendString(bb);
	memset(bb, 0,  sizeof(bb));
	sprintf(bb, "NM  Address: "); 
        net_conf_get_nm_string(bb, 100);
	sendString(bb);
	memset(bb, 0,  sizeof(bb));
	sprintf(bb, "GW  Address: "); 
        net_conf_get_gw_string(bb, 100);
	sendString(bb);
	memset(bb, 0,  sizeof(bb));
// this one I still need to implement... 
	sprintf(bb, "DNS Address: %d.%d.%d.%d\r\n", dns_address[0],dns_address[1],dns_address[2],dns_address[3]); 
	sendString(bb);
	memset(bb, 0,  sizeof(bb));

	while(1)
    {
        
		if(timer_expired(&sensor_timer))
        {
            timer_reset(&sensor_timer);
			//read_sensors();
        }

		uip_len = network_read();
        if(uip_len > 0)
        {
            if(BUF->type == htons(UIP_ETHTYPE_IP))
            {
                uip_arp_ipin(); // arp seems to have issues w/o this
                uip_input();
                if(uip_len > 0)
                {
                    uip_arp_out();
                    network_send();
                }
            }
            else if(BUF->type == htons(UIP_ETHTYPE_ARP))
            {
                uip_arp_arpin(); // this is correct
                if(uip_len > 0)
                {
                    network_send();
                }
            }
        }
        else if(timer_expired(&periodic_timer))
        {
            timer_reset(&periodic_timer);

            for(i = 0; i < UIP_CONNS; i++)
            {
                uip_periodic(i);
                if(uip_len > 0)
                {
                    uip_arp_out();
                    network_send();
                }
            }

            #if UIP_UDP
            for(i = 0; i < UIP_UDP_CONNS; i++)
            {
                uip_udp_periodic(i);
                if(uip_len > 0)
                {
                    uip_arp_out();
                    network_send();
                }
            }
            #endif /* UIP_UDP */

            if(timer_expired(&arp_timer))
            {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }
        else if (net_conf_is_dhcpc() && timer_expired(&dhcp_timer))
        {
            timer_reset(&dhcp_timer);
            dhcpc_renew();
        }
    }

    return 0;
}

#if UIP_CONF_LOGGING == 1
void uip_log(char *m)
{
//    sendString(m);
    printf("%s", m);
    //TODO: Get debug information out here somehow, does anybody know a smart way to do that?
}
#endif

/*---------------------------------------------------------------------------*/


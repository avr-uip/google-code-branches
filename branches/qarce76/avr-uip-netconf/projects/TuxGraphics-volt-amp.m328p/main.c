#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "timer.h"

#include "uip_arp.h"
#include "network.h"
#include "enc28j60.h"
#include "apps-conf.h"
#include <string.h>
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])



//EEPROM parameters (TCP/IP parameters)

struct timer dhcp_timer;

int main(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

	network_init();

	CLKPR = (1<<CLKPCE);	//Change prescaler
	CLKPR = (1<<CLKPS0);	//Use prescaler 2
	enc28j60Write(ECOCON, 1 & 0x7);	//Get a 25MHz signal from enc28j60

	int i;
	uip_ipaddr_t ipaddr;
	struct timer periodic_timer, arp_timer;

	clock_init();

	timer_set(&periodic_timer, CLOCK_SECOND);
	timer_set(&arp_timer, CLOCK_SECOND * 10);

	uip_init();
    // must be done or sometimes arp doesn't work
    uip_arp_init();
	
    // start up the webserver
    //httpd_init();
	//simple_httpd_init();
	telnetd_init();


	while(1){
		uip_len = network_read();


		if(uip_len > 0) {
			if(BUF->type == htons(UIP_ETHTYPE_IP)){
				uip_arp_ipin(); // arp seems to have issues w/o this
				uip_input();
				if(uip_len > 0) {
					uip_arp_out();
					network_send();
				}
			}else if(BUF->type == htons(UIP_ETHTYPE_ARP)){
				uip_arp_arpin(); // this is correct
				if(uip_len > 0){
					network_send();
				}
			}

		}else if(timer_expired(&periodic_timer)) {
			timer_reset(&periodic_timer);


			for(i = 0; i < UIP_CONNS; i++) {
				uip_periodic(i);
				if(uip_len > 0) {
					uip_arp_out();
					network_send();
				}
			}

			#if UIP_UDP
			for(i = 0; i < UIP_UDP_CONNS; i++) {
				uip_udp_periodic(i);
				if(uip_len > 0) {
					uip_arp_out();
					network_send();
				}
			}
			#endif /* UIP_UDP */

			if(timer_expired(&arp_timer)) {
				timer_reset(&arp_timer);
				uip_arp_timer();
			}
		} else if (_enable_dhcp && timer_expired(&dhcp_timer)) {
            // for now turn off the led when we start the dhcp process
            //led_low();
            dhcpc_renew();
            timer_reset(&dhcp_timer);
        }
/// 		while(network_sending()) {asm("nop");}; //wait untill packet is sent away
	}
	return 0;
}

#if UIP_CONF_LOGGING == 1
void uip_log(char *m)
{
	sendString(m);
	//TODO: Get debug information out here somehow, does anybody know a smart way to do that?
}
#endif

/*---------------------------------------------------------------------------*/


void dhcpc_configured(const struct dhcpc_state *s)
{
    uip_ipaddr_t addr;

    // byte swap the network info
    _ip_addr[0] = (s->ipaddr[0]);
    _ip_addr[1] = (s->ipaddr[0]) >> 8;
    _ip_addr[2] = (s->ipaddr[1]);
    _ip_addr[3] = (s->ipaddr[1]) >> 8;

    _net_mask[0] = (s->netmask[0]);
    _net_mask[1] = (s->netmask[0]) >> 8;
    _net_mask[2] = (s->netmask[1]);
    _net_mask[3] = (s->netmask[1]) >> 8;

    _gateway[0] = (s->default_router[0]);
    _gateway[1] = (s->default_router[0]) >> 8;
    _gateway[2] = (s->default_router[1]);
    _gateway[3] = (s->default_router[1]) >> 8;

#ifdef DHCP_DEBUG
    eeprom_write_block (_ip_addr, &ee_ip_addr, 4);
    eeprom_write_block (_net_mask,&ee_net_mask,4);
    eeprom_write_block (_gateway, &ee_gateway, 4);
#endif
    // re-init just in case
	uip_setethaddr(my_eth_addr);

    // set ip
    uip_ipaddr(&addr, _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
    uip_sethostaddr(&addr);
    
    // set netmask
    uip_ipaddr(&addr,_net_mask[0], _net_mask[1], _net_mask[2], _net_mask[3]);
    uip_setnetmask(&addr);

    // set gateway
    uip_ipaddr(&addr,_gateway[0], _gateway[1], _gateway[2], _gateway[3]);
    uip_setdraddr(&addr);

//  code to use dhcp server lease time removed due to uint16_t overflow
//  issues with calculating the time.  Just use 5 minutes instead.  
    timer_set(&dhcp_timer, 5 * 60 * CLOCK_SECOND);

#ifdef DHCP_DEBUG
    // for now turn on the led when we get an ip
    led_high();
#endif
}

/*---------------------------------------------------------------------------*/

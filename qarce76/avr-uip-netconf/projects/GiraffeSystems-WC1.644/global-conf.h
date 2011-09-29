//Project specific configurations
#ifndef __GLOBAL_CONF_H__
#define __GLOBAL_CONF_H__

#include "usart.h"

#ifdef PORTB1
//Led on tuxgraphics board
#define led_conf()      DDRB |= (1<<DDB1)
#define led_low()       PORTB |= (1<<PORTB1)
#define led_high()      PORTB &= ~(1<<PORTB1)
#define led_blink()     PORTB ^= (1<<PORTB1)
#else
//Led on tuxgraphics board
#define led_conf()      DDRB |= (1<<DDB1)
#define led_low()       PORTB |= (1<<PB1)
#define led_high()      PORTB &= ~(1<<PB1)
#define led_blink()     PORTB ^= (1<<PB1)
#endif


//Define frequency
#define F_CPU 12500000UL
//Console setting speed based on:
//  http://www.chip45.com/download/ATxmega_Baud_Rate_Calculator_v1.0.xls.zip
// link was found on:
//  http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=70673 
#define CONSOLE_SPEED_19200 40
#define CONSOLE_SPEED_9600 80

//Mac adress definition for enc28j60
#define ETHADDR0		0x00
#define ETHADDR1		0xbd
#define ETHADDR2		0x3b
#define ETHADDR3		0x33
#define ETHADDR4		0x05
#define ETHADDR5		0x71
//Mac adress definition for uip
#define UIP_ETHADDR0    ETHADDR0
#define UIP_ETHADDR1    ETHADDR1
#define UIP_ETHADDR2    ETHADDR2
#define UIP_ETHADDR3    ETHADDR3
#define UIP_ETHADDR4    ETHADDR4
#define UIP_ETHADDR5    ETHADDR5

#define USE_DHCP 0
#define UIP_IPADDR0 192
#define UIP_IPADDR1 168
#define UIP_IPADDR2 2
#define UIP_IPADDR3 77
#define UIP_NETMASK0 255
#define UIP_NETMASK1 255
#define UIP_NETMASK2 255
#define UIP_NETMASK3 0
#define UIP_DRIPADDR0 192
#define UIP_DRIPADDR1 168
#define UIP_DRIPADDR2 2
#define UIP_DRIPADDR3 1


// ENC28J60 SPI port
#define ENC28J60_SPI_PORT		PORTB
#define ENC28J60_SPI_DDR		DDRB


#if defined(__AVR_ATmega644__)||defined(__AVR_ATmega644P__)
  #define ENC28J60_SPI_SCK        PORTB7
  #define ENC28J60_SPI_MOSI       PORTB5
  #define ENC28J60_SPI_MISO       PORTB6
  #define ENC28J60_SPI_SS         PORTB4
  #define ENC28J60_CONTROL_CS      PORTB4
#else
  #error "NIC SPI PORT PINS NOT DEFINED"
#endif

#define ENC28J60_ENABLE_DUPLEX  1

// ENC28J60 control port
#define ENC28J60_CONTROL_PORT	PORTB
#define ENC28J60_CONTROL_DDR	DDRB

// BEGIN - 74x595 setup
// SER pin 14: set/clear bit on serial data pin
#define S74595_0 PORTC&=~(1<<PORTC5)
#define S74595_1 PORTC|=(1<<PORTC5)
// RCLK pin 12:
#define S74595_RCLKDOWN PORTC&=~(1<<PORTC4)
#define S74595_RCLKUP PORTC|=(1<<PORTC4)
// SRCLK pin 11:
#define S74595_CLOCKDOWN PORTC&=~(1<<PORTC3)
#define S74595_CLOCKUP PORTC|=(1<<PORTC3)
// END - 74x595 setup

// timezone hard code to pacific time for now
#define NTP_TZ 8
#define NTP_REPEAT 10
#define NTP_REQ_CYCLE 20

// Real Time Clock settings
#define RTC_CE PORTA2 
#define RTC_SCK PORTA0
#define RTC_PORT PORTA
#define RTC_DDR DDRA
#define RTC_IO_PORT PORTA
#define RTC_IO_PIN PINA
#define RTC_IO_DDR DDRA
#define RTC_IO PORTA1


//Include uip.h gives all the uip configurations in uip-conf.h
#include "uip.h"

#endif /*__GLOBAL_CONF_H__*/

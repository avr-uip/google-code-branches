#include <avr/io.h>
#include <config.h>
#include "soft_spi.h"
#include <util/delay_basic.h>
#include <util/delay.h>


/* BASIC SPI STUFF */

/*
requires definitions of ATMEL ports:

	SPI_PORT		Port for the SPI signals eg PORTB
	SPI_DDR 		DDR for the SPI signals eg DDRB
	SPI_PIN 		Input Port for the SPI signals eg PINB

	SPI_SCK 		Clock from master to slave
	SPI_MOSI   		Data from Master to slave
	SPI_MISO   		Data from slave to master


*/





/* SPI mode 0 where data to slave is latched on leading edge of SCK, and data
from slave to master is latched by master on fallling edge of SCK */

inline void spi_clock_up_down() {
	SPI_PORT |= (1<<SPI_SCK);
	_delay_us(2);
	SPI_PORT &= ~(1<<SPI_SCK);
}


uint8_t read_spi_byte() {

	uint8_t i;
	uint8_t bitresult;
	uint8_t result;
	SPI_PORT &= ~(1<<SPI_SCK);
	for (i=0;i<8;i++) {
		_delay_us(1);
		bitresult = ((SPI_PIN >> SPI_MISO) & 0x01);
		result = (result << 1) | bitresult;
		spi_clock_up_down();
	}	
	return result;
}


uint16_t read_spi_2byte() {
	uint8_t i;
	uint8_t bitresult;
	uint16_t result;
	SPI_PORT &= ~(1<<SPI_SCK);
	for (i=0;i<16;i++) {
		_delay_us(2);
		bitresult = ((SPI_PIN >> SPI_MISO) & 0x01);
		result = (result << 1) | bitresult;
		spi_clock_up_down();
	}	
	return result;
}


void write_spi_byte(uint8_t data) {

/* enter and leave with with SCK low */
	uint8_t bitpos;
	uint8_t bitvalue;
	SPI_PORT &= ~(1<<SPI_SCK);
	for (bitpos=8;bitpos>0;--bitpos) {
		bitvalue = ((data >> (bitpos-1)) & 0x01);
		if (bitvalue) {
			SPI_PORT |= 1<<SPI_MOSI;
		} else {
			SPI_PORT &= ~(1<<SPI_MOSI);
		}
		_delay_us(1); 
		spi_clock_up_down();
	}
}


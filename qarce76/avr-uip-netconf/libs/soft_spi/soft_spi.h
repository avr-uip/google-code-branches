#include <avr/io.h>
#include <config.h>

void clock_up_down();
void wait12uS();
uint8_t read_spi_byte();
uint16_t read_spi_2byte();
void write_spi_byte(uint8_t data);

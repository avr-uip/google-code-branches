#ifndef __EEPROM_ADDR_H__
#define __EEPROM_ADDR_H__

#define eeprom_fixed

#define serial_start 0x0000
#define serial_len 10

#define mac_start 0x000A
#define mac_len 6

#define ip_address_start 0x0010
#define ip_address_len 4

#define nm_address_start 0x0014
#define nm_address_len 4

#define gw_address_start 0x0018
#define gw_address_len 4

#define dns_address_start 0x001C
#define dns_address_len 4

#define dhcp_enabled_start 0x0020

#endif /*__EEPROM_ADDR_H__*/
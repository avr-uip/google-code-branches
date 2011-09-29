/***************************************************************************
 * en28j60.c: ENC28J60 driver for ATmega88.
 * Copyright (C) 2007 Michael C McTernan,
 *    Michael.McTernan.2001@cs.bris.ac.uk
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**************************************************************************
 * Includes
 **************************************************************************/

#include <stdint.h>
#include "enc28j60.h"

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Prototypes
 **************************************************************************/

/**************************************************************************
 * Variables
 **************************************************************************/

/**************************************************************************
 * Local Functions
 **************************************************************************/

/**************************************************************************
 * Global Functions
 **************************************************************************/
#if 1
void enc28j60Debug(uint8_t v1, uint8_t v2, uint8_t v3,
                   uint8_t v4, uint8_t v5, uint8_t v6)
{
}
#endif


void enc28j60Init(void)
{
}

void enc28j60PowerUp(void)
{
}

void enc28j60PowerDown(void)
{
}

void enc28j60HandleEvents (void (*linkEv)(void *), void *linkData,
                           void (*packetEv)(void *), void *packetData)
{
}

void enc28j60TxPktStart(void)
{
}

void enc28j60TxPktWriteMacAddr(macaddrtype_t type)
{
}


void enc28j60TxPktAppend(uint8_t n, const void *data)
{
}

void enc28j60TxPktFill(uint16_t n, uint8_t pattern)
{
}

void enc28j60TxPktAppendRx(uint16_t n)
{
}

void enc28j60TxPktSend(void)
{
}

boolean_t enc28j60RxPktStart(void)
{
}

void enc28j60RxPktRead(uint8_t n, void *data)
{
}

boolean_t enc28j60RxPktCmp(uint8_t n, const void *data)
{
}

void enc28j60RxPktFree(void)
{
}

uint8_t enc28j60GetRevId(void)
{
}

boolean_t enc28j60IsLinkUp(void)
{
}

void enc28j60GetMac(macaddr_t *const mac)
{
}

void enc28j60RxPktRewind(void)
{
}

/* END OF FILE */

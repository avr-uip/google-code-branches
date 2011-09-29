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

#ifndef ENC28J60_H
#define ENC28J60_H

/**************************************************************************
 * Includes
 **************************************************************************/

#include <stdint.h>
#include "main.h"
#include "eth.h"

/**************************************************************************
 * Types
 **************************************************************************/

/** MAC address types.
 */
typedef enum
{
    MAC_ADDR_TYP_LOCAL,  /**< The local MAC address of this device. */
    MAC_ADDR_TYP_BCAST,  /**< The broadcast MAC address. */
    MAC_ADDR_TYP_ZERO    /**< All zeros */
}
macaddrtype_t;

/**************************************************************************
 * Prototypes
 **************************************************************************/

void      enc28j60Init              (void);
void      enc28j60InitSpi           (void);
void      enc28j60PowerUp           (void);
void      enc28j60PowerDown         (void);

void      enc28j60HandleEvents      (void (*linkEv)(void *), void *linkData,
                                     void (*packetEv)(void *), void *packetData);

boolean_t enc28j60RxPktStart        (void);
void      enc28j60RxPktRead         (uint8_t n, void *data);
boolean_t enc28j60RxPktCmp          (uint8_t n, const void *data);
boolean_t enc28j60RxPktCmpPrgMem    (uint8_t n, const void *data);
#define   enc28j60RxPktSkip(n)      enc28j60RxPktRead(n, NULL)
boolean_t enc28j60RxEop             (void);
uint16_t  enc28j60RxPktOffset       (void);
void      enc28j60RxPktRewind       (void);
void      enc28j60RxPktFree         (void);


void      enc28j60TxPktStart        (void);
void      enc28j60TxPktWriteMacAddr (macaddrtype_t type);
void      enc28j60TxPktAppend       (uint8_t n, const void *data);
void      enc28j60TxPktAppendRx     (uint16_t n);
void      enc28j60TxPktAppendPrgMem (uint8_t n, const void *data);
void      enc28j60TxPktFill         (uint16_t n, uint8_t pattern);
void      enc28j60TxPktSend         (void);

uint16_t  enc28j60TxPktGetPtr       (void);
void      enc28j60TxPktSetPtr       (uint16_t v);

uint8_t   enc28j60GetRevId          (void);

boolean_t enc28j60IsLinkUp          (void);

void      enc28j60GetMac            (macaddr_t *const mac);

#if defined(ON_PC)
#define enc28j60TxPktAppendPrgMem enc28j60TxPktAppend
#define enc28j60RxPktCmpPrgMem    enc28j60RxPktCmp
#endif

#endif /* ENC28J60_H */

/* END_OF_FILE */

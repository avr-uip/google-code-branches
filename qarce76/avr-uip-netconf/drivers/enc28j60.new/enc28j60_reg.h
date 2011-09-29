/***************************************************************************
 * en28j60_reg.h: ENC28J60 register definitions.
 * Original Copyright (C) 2005 Pascal Stang
 * Modifications Copyright (C) 2007 Michael C McTernan,
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

#ifndef ENC28J60_REG_H
#define ENC28J60_REG_H

/**************************************************************************
 * SPI opcodes
 **************************************************************************/

#define OP_READ_CTRL_REG  0x00
#define OP_READ_BUF_MEM   0x3A
#define OP_WRITE_CTRL_REG 0x40
#define OP_WRITE_BUF_MEM  0x7A
#define OP_BIT_FIELD_SET  0x80
#define OP_BIT_FIELD_CLR  0xA0
#define OP_SOFT_RESET     0xFF

/**************************************************************************
 * Register Masks
 **************************************************************************/

/** Mask to give the register address from constants defined in this file. */
#define REG_ADDR_MASK       0x1F

/** Mask to give the bank of a register from constants defined in this file. */
#define REG_BANK_MASK       0x60

/** Marker bit for MAC and MII registers that need an extra dummy read. */
#define REG_EXTRA_RD_BIT    0x80

/** Shift to apply to bank number before writing to ECON1. */
#define REG_BANK_SHIFT         5

/**************************************************************************
 * Common registers, accessible in all banks
 **************************************************************************/

#define EIE         0x1B
#define EIR         0x1C
#define ESTAT       0x1D
#define ECON2       0x1E
#define ECON1       0x1F


/**************************************************************************
 * Bank 0 Registers
 **************************************************************************/

#define ERDPTL      (0x00|0x00)
#define ERDPTH      (0x01|0x00)
#define EWRPTL      (0x02|0x00)
#define EWRPTH      (0x03|0x00)
#define ETXSTL      (0x04|0x00)
#define ETXSTH      (0x05|0x00)
#define ETXNDL      (0x06|0x00)
#define ETXNDH      (0x07|0x00)
#define ERXSTL      (0x08|0x00)
#define ERXSTH      (0x09|0x00)
#define ERXNDL      (0x0A|0x00)
#define ERXNDH      (0x0B|0x00)
#define ERXRDPTL    (0x0C|0x00)
#define ERXRDPTH    (0x0D|0x00)
#define ERXWRPTL    (0x0E|0x00)
#define ERXWRPTH    (0x0F|0x00)
#define EDMASTL     (0x10|0x00)
#define EDMASTH     (0x11|0x00)
#define EDMANDL     (0x12|0x00)
#define EDMANDH     (0x13|0x00)
#define EDMADSTL    (0x14|0x00)
#define EDMADSTH    (0x15|0x00)
#define EDMACSL     (0x16|0x00)
#define EDMACSH     (0x17|0x00)


/**************************************************************************
 * Bank 1 Registers
 **************************************************************************/

#define EHT0        (0x00|0x20)
#define EHT1        (0x01|0x20)
#define EHT2        (0x02|0x20)
#define EHT3        (0x03|0x20)
#define EHT4        (0x04|0x20)
#define EHT5        (0x05|0x20)
#define EHT6        (0x06|0x20)
#define EHT7        (0x07|0x20)
#define EPMM0       (0x08|0x20)
#define EPMM1       (0x09|0x20)
#define EPMM2       (0x0A|0x20)
#define EPMM3       (0x0B|0x20)
#define EPMM4       (0x0C|0x20)
#define EPMM5       (0x0D|0x20)
#define EPMM6       (0x0E|0x20)
#define EPMM7       (0x0F|0x20)
#define EPMCSL      (0x10|0x20)
#define EPMCSH      (0x11|0x20)
#define EPMOL       (0x14|0x20)
#define EPMOH       (0x15|0x20)
#define EWOLIE      (0x16|0x20)
#define EWOLIR      (0x17|0x20)
#define ERXFCON     (0x18|0x20)
#define EPKTCNT     (0x19|0x20)


/**************************************************************************
 * Bank 2 Registers
 **************************************************************************/

#define MACON1      (0x00|0x40|REG_EXTRA_RD_BIT)
#define MACON2      (0x01|0x40|REG_EXTRA_RD_BIT)
#define MACON3      (0x02|0x40|REG_EXTRA_RD_BIT)
#define MACON4      (0x03|0x40|REG_EXTRA_RD_BIT)
#define MABBIPG     (0x04|0x40|REG_EXTRA_RD_BIT)
#define MAIPGL      (0x06|0x40|REG_EXTRA_RD_BIT)
#define MAIPGH      (0x07|0x40|REG_EXTRA_RD_BIT)
#define MACLCON1    (0x08|0x40|REG_EXTRA_RD_BIT)
#define MACLCON2    (0x09|0x40|REG_EXTRA_RD_BIT)
#define MAMXFLL     (0x0A|0x40|REG_EXTRA_RD_BIT)
#define MAMXFLH     (0x0B|0x40|REG_EXTRA_RD_BIT)
#define MAPHSUP     (0x0D|0x40|REG_EXTRA_RD_BIT)
#define MICON       (0x11|0x40|REG_EXTRA_RD_BIT)
#define MICMD       (0x12|0x40|REG_EXTRA_RD_BIT)
#define MIREGADR    (0x14|0x40|REG_EXTRA_RD_BIT)
#define MIWRL       (0x16|0x40|REG_EXTRA_RD_BIT)
#define MIWRH       (0x17|0x40|REG_EXTRA_RD_BIT)
#define MIRDL       (0x18|0x40|REG_EXTRA_RD_BIT)
#define MIRDH       (0x19|0x40|REG_EXTRA_RD_BIT)


/**************************************************************************
 * Bank 3 Registers
 **************************************************************************/

#define MAADR5      (0x00|0x60|REG_EXTRA_RD_BIT)
#define MAADR6      (0x01|0x60|REG_EXTRA_RD_BIT)
#define MAADR3      (0x02|0x60|REG_EXTRA_RD_BIT)
#define MAADR4      (0x03|0x60|REG_EXTRA_RD_BIT)
#define MAADR1      (0x04|0x60|REG_EXTRA_RD_BIT)
#define MAADR2      (0x05|0x60|REG_EXTRA_RD_BIT)
#define EBSTSD      (0x06|0x60)
#define EBSTCON     (0x07|0x60)
#define EBSTCSL     (0x08|0x60)
#define EBSTCSH     (0x09|0x60)
#define MISTAT      (0x0A|0x60|REG_EXTRA_RD_BIT)
#define EREVID      (0x12|0x60)
#define ECOCON      (0x15|0x60)
#define EFLOCON     (0x17|0x60)
#define EPAUSL      (0x18|0x60)
#define EPAUSH      (0x19|0x60)

/**************************************************************************
 * Phy Registers
 **************************************************************************/

#define PHCON1          0x00
#define PHSTAT1         0x01
#define PHHID1          0x02
#define PHHID2          0x03
#define PHCON2          0x10
#define PHSTAT2         0x11
#define PHIE            0x12
#define PHIR            0x13
#define PHLCON          0x14

/**************************************************************************
 * Register bits
 **************************************************************************/

#define ERXFCON_UCEN    0x80
#define ERXFCON_ANDOR   0x40
#define ERXFCON_CRCEN   0x20
#define ERXFCON_PMEN    0x10
#define ERXFCON_MPEN    0x08
#define ERXFCON_HTEN    0x04
#define ERXFCON_MCEN    0x02
#define ERXFCON_BCEN    0x01

#define EIE_INTIE       0x80
#define EIE_PKTIE       0x40
#define EIE_DMAIE       0x20
#define EIE_LINKIE      0x10
#define EIE_TXIE        0x08
#define EIE_WOLIE       0x04
#define EIE_TXERIE      0x02
#define EIE_RXERIE      0x01

#define EIR_PKTIF       0x40
#define EIR_DMAIF       0x20
#define EIR_LINKIF      0x10
#define EIR_TXIF        0x08
#define EIR_WOLIF       0x04
#define EIR_TXERIF      0x02
#define EIR_RXERIF      0x01

#define ESTAT_INT       0x80
#define ESTAT_LATECOL   0x10
#define ESTAT_RXBUSY    0x04
#define ESTAT_TXABRT    0x02
#define ESTAT_CLKRDY    0x01

#define ECON2_AUTOINC   0x80
#define ECON2_PKTDEC    0x40
#define ECON2_PWRSV     0x20
#define ECON2_VRPS      0x08

#define ECON1_TXRST     0x80
#define ECON1_RXRST     0x40
#define ECON1_DMAST     0x20
#define ECON1_CSUMEN    0x10
#define ECON1_TXRTS     0x08
#define ECON1_RXEN      0x04
#define ECON1_BSEL1     0x02
#define ECON1_BSEL0     0x01

#define MACON1_LOOPBK   0x10
#define MACON1_TXPAUS   0x08
#define MACON1_RXPAUS   0x04
#define MACON1_PASSALL  0x02
#define MACON1_MARXEN   0x01

#define MACON2_MARST    0x80
#define MACON2_RNDRST   0x40
#define MACON2_MARXRST  0x08
#define MACON2_RFUNRST  0x04
#define MACON2_MATXRST  0x02
#define MACON2_TFUNRST  0x01

#define MACON3_PADCFG2  0x80
#define MACON3_PADCFG1  0x40
#define MACON3_PADCFG0  0x20
#define MACON3_TXCRCEN  0x10
#define MACON3_PHDRLEN  0x08
#define MACON3_HFRMLEN  0x04
#define MACON3_FRMLNEN  0x02
#define MACON3_FULDPX   0x01

#define MICMD_MIISCAN   0x02
#define MICMD_MIIRD     0x01

#define MISTAT_NVALID   0x04
#define MISTAT_SCAN     0x02
#define MISTAT_BUSY     0x01

#define PHCON1_PRST     0x8000
#define PHCON1_PLOOPBK  0x4000
#define PHCON1_PPWRSV   0x0800
#define PHCON1_PDPXMD   0x0100

#define PHSTAT1_PFDPX   0x1000
#define PHSTAT1_PHDPX   0x0800
#define PHSTAT1_LLSTAT  0x0004
#define PHSTAT1_JBSTAT  0x0002

#define PHSTAT2_TXSTAT  0x2000
#define PHSTAT2_RXSTAT  0x1000
#define PHSTAT2_COLSTAT 0x0800
#define PHSTAT2_LSTAT   0x0400
#define PHSTAT2_DPXSTAT 0x0200
#define PHSTAT2_PLRITY  0x0010

#define PHCON2_FRCLINK  0x4000
#define PHCON2_TXDIS    0x2000
#define PHCON2_JABBER   0x0400
#define PHCON2_HDLDIS   0x0100

#define PHIE_PLNKIE     0x0010
#define PHIE_PGEIE      0x0002

#define PKTCTRL_PHUGEEN   0x08
#define PKTCTRL_PPADEN    0x04
#define PKTCTRL_PCRCEN    0x02
#define PKTCTRL_POVERRIDE 0x01

#endif /* ENC28J60_REG_H */

/* END OF FILE */


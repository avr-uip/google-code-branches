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

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "enc28j60.h"
#include "enc28j60_reg.h"
#include "event.h"

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

/* Ethernet constants */
#define ETHERNET_MIN_PACKET_LENGTH  0x3C
#define ETHERNET_HEADER_LENGTH      0x0E
#define ETHERNET_CRC_BYTES             4

/** Last byte of the rx buffer.
 */
#define BUF_RX_END  0x0fff

/**************************************************************************
 * Macros
 **************************************************************************/

#define M_LoByte(v) ((v) & 0xff)
#define M_HiByte(v) (((v)>> 8) & 0xff)

/**************************************************************************
 * Prototypes
 **************************************************************************/

static void    regClrBits(uint8_t address, uint8_t data);
static void    regSetBits(uint8_t address, uint8_t data);
static uint8_t regRead   (uint8_t address);
static void    regWrite  (uint8_t address, uint8_t data);
static void    writeOp   (uint8_t address, uint8_t data, uint8_t op);

/**************************************************************************
 * Variables
 **************************************************************************/

/** Initialisation words for the device, stored in FLASH.
 */
static const struct
{
    uint8_t addr, data;
}
PROGMEM initwords[] =
{
    /* Rx buffer setup */
    {ERXSTL, 0x00},   /* Start Rx buffer starts at 0.  This */
    {ERXSTH, 0x00},   /*  also sets the ERXWRPT. */
                      /* Eratta for rev B4 says
                         "Sometimes, when ERXST or ERXND is written to,
                         the exact value, 0000h, is stored in the internal
                         receive Write Pointer instead of the ERXST address."
                         The workaround is to place the write address at 0. */

    {ERXNDL, M_LoByte(BUF_RX_END)},   /* End Rx buffer upto and including 0x0fff = 4095 */
    {ERXNDH, M_HiByte(BUF_RX_END)},

    {ERXRDPTL, 0x00}, /* Rx read pointer at end */
    {ERXRDPTH, 0x00},

    /* Setup filter to allow broadcast or addressed to our MAC */
    {ERXFCON, ERXFCON_UCEN | ERXFCON_BCEN | ERXFCON_CRCEN},
    /*{ERXFCON, 0},*/  /* Promiscuous */

    /* MAC initialisation */
    {MACON2, MACON2_MARST},   /* Bring out of reset */
    {MACON1, MACON1_MARXEN},  /* Enable Rx */
    {MACON3, MACON3_PADCFG0 | /* Pad to 60bytes & add CRC*/
             MACON3_TXCRCEN},

    {MAMXFLL, M_LoByte(1518)},  /* Max frame length */
    {MAMXFLH, M_HiByte(1518)},

    {MABBIPG, 0x12}, /* Half duplex setting */
    {MAIPGL,  0x12}, /* Half duplex setting */
    {MAIPGH,  0x0c}, /* Half duplex setting */

    /* The MAC address */
    {MAADR1, 0xc0 },
    {MAADR2, 0x02 },
    {MAADR3, 0xca },
    {MAADR4, 0xfe },
    {MAADR5, 0xba },
    {MAADR6, 0xbe }
};


#define INIT_MAC_ADDR_START ((sizeof(initwords) / sizeof(initwords[0])) - sizeof(macaddr_t))

/** Count of unprocessed bytes in the receive buffer.
 * This counts the number of unprocessed bytes in the currently being read
 * packet.  If the application code reads beyond the end of the packet,
 * this will be negative.
 */
static int16_t rxPktBytesLength;

/** Count of bytes that have been processed from the receive buffer.
 */
static int16_t rxPktBytesConsumed;

/**************************************************************************
 * Local Functions
 **************************************************************************/

/** Wait for the SPI interface to serialise the last written byte.
 */
static void waitSpi(void)
{
    while(!(SPSR & (1<<SPIF)));
}

/** Assert or clear CS, and configure SPI if required.
 *
 * \param[in] switchOn  If TRUE, set CS and setup SPI if needed.  Otherwise
 *                       clear CS.
 */
static void selectDevice(boolean_t switchOn)
{
    if(switchOn)
    {
        /* Check if the SPI is enabled */
        if((PRR & (1<<PRSPI)) != 0)
        {
            /* Enable SPI block */
            PRR &= ~(1<<PRSPI);

            /* Init SPI master mode, and set clock as Fosc/2 = 4 MHz */
            SPCR  = (1<<SPE) | (1<<MSTR);
            SPSR |= (1<<SPI2X);

            /* Note: Errata for enc28j60, rev 0x04 says:
             *       "Module: MAC Interface
             *        When the SPI clock from the host microcontroller
             *        is run at frequencies of less than 8 MHz, reading or
             *        writing to the MAC registers may be unreliable."
             *
             * This means that the SPI may not be running fast enough for
             *  some enc28j70 parts.
             */

        }

        /* Lower the CS line to select chip */
        PORTB &= ~(1<<PB2);
    }
    else
    {
        /* Raise the CS line to deselect chip */
        PORTB |= (1<<PB2);
    }
}


/** Update bank selection bits in ECON1 if needed.
 * After comparing the currently selected bank with the bank needed for
 * accessing some address, ECON1 maybe modified such that the bank required
 * for accessing \a address is selected.  If the supplied address is already
 * mapped or is a common register, no update of ECON1 is programmed.
 *
 * \param[in] address  The address of the register that needs mapping.
 */
static void bankSelect(uint8_t address)
{
    static uint8_t curBank = 0x00; /* Device reset value */
    const  uint8_t addBank = address & REG_BANK_MASK;

    /* Check if the bank needs switching */
    if(addBank != curBank)
    {
        /* Clear the bank select bits */
        regClrBits(ECON1, ECON1_BSEL0 | ECON1_BSEL1);

        /* Set the desired selection.
         *  BSEL0 is bit 1 and BSEL1 is bit 2.
         */
        regSetBits(ECON1, addBank >> REG_BANK_SHIFT);

        /* Store the current bank selection */
        curBank = addBank;
    }
}


/** Perform a write operation to some register.
 * This performs a write operation to modify the value of some register.
 *
 * \param[in] address  The register address, which also encode the bank.
 * \param[in] data     The word to write.
 * \param[in] op       The SPI operation to perform on the register, which
 *                      must be from one of the \a OP_ macros.
 */
static void writeOp(const uint8_t address, const uint8_t data, const uint8_t op)
{
    const uint8_t addr = address & REG_ADDR_MASK;

    /* Select the register bank if needed */
    if(addr < EIE) bankSelect(address);

    /* Set chip select */
    selectDevice(TRUE);

    /* Write the command */
    SPDR = op | addr;
    waitSpi();

    /* Write the data */
    SPDR = data;
    waitSpi();

    /* Clear chip select */
    selectDevice(FALSE);
}


/** Perform a read operation on the enc28j60.
 */
static uint8_t readOp(uint8_t address, uint8_t op)
{
    const uint8_t addr = address & REG_ADDR_MASK;
    uint8_t       result;

    /* Select the register bank if needed */
    if(addr < EIE) bankSelect(address);

    /* Set chip select */
    selectDevice(TRUE);

    /* Send read command */
    SPDR = op | addr;
    waitSpi();

    /* Clock another 8-bits for the result */
    SPDR = 0x00;
    waitSpi();

    /* If a MAC or MII register, a dummy byte will have been read */
    if(address & REG_EXTRA_RD_BIT)
    {
        /* Clock another 8-bits for the result */
        SPDR = 0x00;
    }

    waitSpi();

    /* Get the result before disabling SPI */
    result = SPDR;

    /* Clear chip select */
    selectDevice(FALSE);

    /* Return the byte */
    return result;
}


static uint8_t regRead(uint8_t address)
{
    return readOp(address, OP_READ_CTRL_REG);
}


static void regWrite(uint8_t address, uint8_t data)
{
    writeOp(address, data, OP_WRITE_CTRL_REG);
}


static void regWritePair(uint8_t addressL, uint16_t data)
{
    regWrite(addressL, M_LoByte(data));
    regWrite(addressL + 1, M_HiByte(data));
}


static uint16_t regReadPair(uint8_t addressL)
{
    return regRead(addressL) | (regRead(addressL + 1) << 8);
}


static void regSetBits(uint8_t address, uint8_t data)
{
    writeOp(address, data, OP_BIT_FIELD_SET);
}


/** Clear bits in some register.
 * This causes the enc28j60 hardware to NAND \a data with the given register,
 * which must be a ETH register.
 */
static void regClrBits(uint8_t address, uint8_t data)
{
    writeOp(address, data, OP_BIT_FIELD_CLR);
}


/** Wait for some register value to change.
 *
 * \param[in] address The register to poll.
 * \param[in] mask    Bitmask to apply to the register value.
 * \param[in] waitVal Value that \a *address & \a mask must equal before
 *                     the function will return.
 */
static void regWait(uint8_t address, uint8_t mask, uint8_t waitVal)
{
    while((regRead(address) & mask) != waitVal);
}


static void phyWait(void)
{
    do
    {
        _delay_us(10.24f);
    }
    while(regRead(MISTAT) & MISTAT_BUSY);
}


static void phyWrite(uint8_t address, uint16_t data)
{
    /* Set PHY register address */
    regWrite(MIREGADR, address);

    /* Write data, hi byte must be second */
    regWritePair(MIWRL, data);
    phyWait();
}


static uint16_t phyRead(uint8_t address)
{
    /* Setup the read address and start read */
    regWrite(MIREGADR, address);
    regWrite(MICMD, MICMD_MIIRD);

    /* Wait for read to complete */
    phyWait();

    regWrite(MICMD, 0);

    return regRead(MIRDL) | (regRead(MIRDH) << 8);
}


/** Write data to the buffer memory.
 * This writes data into the buffer memory.
 *
 * \param[in] n     The number of bytes to write.
 * \param[in] data  Pointer to data that should be written.
 */
static void bufWrite(uint8_t n, const uint8_t *data)
{
#if 1
    uint8_t *d = (uint8_t *)data;

    /* Compact loop.
     *  This saves a few bytes, but doesn't pipeline bytes, so is slower.
     */
    while(n > 0)
    {
         writeOp(0, *d, OP_WRITE_BUF_MEM);
         d++; n--;
    }
#else
    /* Set chip select */
    selectDevice(TRUE);

    /* Write the command */
    SPDR = OP_WRITE_BUF_MEM;
    waitSpi();

    /* Write the data */
    do
    {
        SPDR = *data;
        n--; data++;
        waitSpi();
    }
    while(n > 0);

    /* Clear chip select */
    selectDevice(FALSE);
#endif
}


/** Set the buffer read pointer to the value of the Rx read pointer.
 * Assuming that Rx read pointer points to the start of a packet, this
 * sets the Rx read pointer to the same address such that a packet can
 * be read.
 *
 * \returns  The value of the RX read pointer.
 */
static uint16_t seekBufPktStart(void)
{
    uint16_t v = regReadPair(ERXRDPTL);
    regWritePair(ERDPTL, v);
    return v;
}


/** Interrupt handler for enc28j60 interrupt source.
 *
 * \note Global interrupts are disabled while this executes.
 */
ISR(INT0_vect)
{
    /* Disable this as an interrupt source */
    EIMSK &= ~(1<<INT0);

    /* Set event and prevent sleeping */
    EVENT_REG |= (1<<EV_ETH);
    SMCR &= ~(1<<SE);
}

/**************************************************************************
 * Global Functions
 **************************************************************************/

/** Initialise the enc28j60 device.
 * This performs one time initialisation for the device.
 */
void enc28j60Init(void)
{
    /* Set inputs and outputs:
     *  Out - B0 = RESET
     *        B2 = CS
     *        B3 = MOSI
     *        B5 = SCK
     *  In  - B4 = MISO
     */
    DDRB |= (1<<DDB0) | (1<<DDB2) | (1<<DDB3) | (1<<DDB5);
    DDRB &= ~(1<<DDB4);

    /* Lower RESET, MOSI and SCK.  Device now held in reset.  */
    PORTB &= ~((1<<PB0) | (1<<PB3) | (1<<PB5));

    /* Set CS, so the device is not selected */
    PORTB |= (1<<PB2);

    /* Setup INT0 as an interrupt source */
    DDRD  &= ~(1<DDD2);
    EICRA &= ~((1<<ISC00) | (1<<ISC01)); /* Interrupt when low, CLKio not needed */
    EIMSK |= (1<<INT0);

    /* Bring out of reset */
    PORTB |= (1<<PB0);

    /* Wait for the clock to settle */
    regWait(ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY);

    /* Write the init words */
    for(uint8_t t = 0; t < sizeof(initwords) / sizeof(initwords[0]); t++)
    {
        regWrite(pgm_read_byte_near(&initwords[t].addr),
                 pgm_read_byte_near(&initwords[t].data));
    }

    /* Phy initialisation */
    phyWrite(PHCON2, PHCON2_HDLDIS); /* Prevent loopback of Tx'd packets */

    /* Other initialisation */

    /* Enable link state change interrupts from the PHY */
    phyWrite(PHIE, PHIE_PGEIE | PHIE_PLNKIE);

    /* Global interrupt enable and enable packet Rx and link state interrupts */
    regSetBits(EIE, EIE_INTIE | EIE_PKTIE | EIE_LINKIE);

    /* Enable packet reception */
    regSetBits(ECON1, ECON1_RXEN);
}


/** Power up the enc28j60 device.
 * This brings the device out of power saving mode.
 */
void enc28j60PowerUp(void)
{
    regClrBits(ECON2, ECON2_PWRSV);
    regWait(ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY);
    regSetBits(ECON1, ECON1_RXEN);
}


/** Power down the enc28j60 device.
 * This puts the device into power saving mode.  In this mode packets
 * cannot be transmitted or received, but ETH registers and buffer
 * memory can still be accessed.
 */
void enc28j60PowerDown(void)
{
    regClrBits(ECON1, ECON1_RXEN);
    delay_ms(10);
    regWait(ESTAT, ESTAT_RXBUSY, 0);
    regSetBits(ECON2, ECON2_VRPS);
    regSetBits(ECON2, ECON2_PWRSV);
}


/** Handle any interrupt events for the device.
 * This checks the status of the interrupt flag register and then takes
 * actions to clear the interrupt events.  The passed functions are called
 * when events of certain types are detected, after which the Ethernet
 * interrupt will be re-enabled.
 *
 * \param[in] linkEv    Function to call on link state changes.
 * \param[in] packetEv  Function to call when a packet is received.
 */
void enc28j60HandleEvents (void (*linkEv)(void *), void *linkData,
                           void (*packetEv)(void *), void *packetData)
{
    const uint8_t eir = regRead(EIR);

    /* Check if the link state has changed */
    if(linkEv != NULL && eir & EIR_LINKIF)
    {
        /* Read the PHY interrupt register to clear */
        phyRead(PHIR);

        /* Call link state change handler */
        linkEv(linkData);
    }


    /* Errata for ENC38J60 rev B4 says:
     * "6. Module: Interrupts
     *  The Receive Packet Pending Interrupt Flag
     *  (EIR.PKTIF) does not reliably/accurately report
     *  the status of pending packets."
     *
     * Instead reading EPKTCNT is recommended.
     */
#if 0
    if(eir & EIR_PKTIF)
#else
    if(1)
#endif
    {
        assert(packetEv != NULL);

        /* Call packet handler.
         *  This should call enc28j60RxPktStart() to check that a packet is
         *  buffered, and then process it, eventually calling
         *  enc28j60RxPktFree() such that PKTDEC is set and the interrupt
         *  source cleared.
         */
        packetEv(packetData);
    }

    /* Re-enable the interrupt source */
    EIMSK |= (1<<INT0);
}


/** Setup for starting to write a packet to Tx.
 */
void enc28j60TxPktStart(void)
{
    const uint8_t pktCtrlByte = 0;

    /* Wait until any previous Tx has completed */
    regWait(ECON1, ECON1_TXRTS, 0);

    /* Setup the buffer write pointer */
    regWritePair(EWRPTL, 0x1000);

    /* Reset the transmit logic as per B4 Errata, point 12 */
    regSetBits(ECON1, ECON1_TXRST);
    regClrBits(ECON1, ECON1_TXRST);

    /* Set the Tx start pointer */
    regWritePair(ETXSTL, 0x1000);

    /* Write per packet control byte */
    bufWrite(1, &pktCtrlByte);

    /* Now ready for writing the body */
}


/** Write a 6-byte MAC address to the buffer memory.
 * This writes a MAC address to the packet buffer.  Depending on the type
 * passed, either the local MAC address or a preset address can be written.
 * The MAC address of other devices can be written using enc28j60PktAppend().
 *
 * \param[in] type  The MAC address type to write.
 */
void enc28j60TxPktWriteMacAddr(macaddrtype_t type)
{
    macaddr_t mac;

    switch(type)
    {
        case MAC_ADDR_TYP_LOCAL:
            enc28j60GetMac(&mac);
            bufWrite(sizeof(macaddr_t), mac.b);
            break;

        case MAC_ADDR_TYP_BCAST:
            enc28j60TxPktFill(sizeof(macaddr_t), 0xff);
            break;

        case MAC_ADDR_TYP_ZERO:
            enc28j60TxPktFill(sizeof(macaddr_t), 0x00);
            break;
    }
}


/** Append a buffer of data to the transmit buffer.
 *
 * \param[in] n     Count of bytes to write.
 * \param[in] data  Pointer to first byte to be written.
 */
void enc28j60TxPktAppend(uint8_t n, const void *data)
{
    bufWrite(n, data);
}


/** Append a buffer of data from program memory to the transmit buffer.
 *
 * \param[in] n     Count of bytes to write.
 * \param[in] data  Pointer to first program memory byte to be written.
 */
void enc28j60TxPktAppendPrgMem(uint8_t n, const void *data)
{
    const uint8_t *d = data;

    while(n > 0)
    {
        uint8_t c = pgm_read_byte_near(d);

        bufWrite(1, &c);

        d++;
        n--;
    }
}


/** Fill n-bytes of the transmit buffer with a repeated byte.
 * This writes the \a pattern byte \a n times to the transmit buffer.
 *
 * \param[in] n        Count of bytes to write.
 * \param[in] pattern  The byte to write.
 */
void enc28j60TxPktFill(uint16_t n, uint8_t pattern)
{
    while(n > 0)
    {
        bufWrite(1, &pattern);
        n--;
    }
}


/** Copy data into the Tx buffer from the current Rx pointer.
 *
 * \param[in] n The number of bytes to copy.
 */
void enc28j60TxPktAppendRx(uint16_t n)
{
    /* DMA cannot be used for 1 byte */
    if(n == 1)
    {
        uint8_t b;

        enc28j60RxPktRead(1, &b);
        enc28j60TxPktAppend(1, &b);
    }
    else if(n > 0)
    {
        uint16_t addr;

        /* Get the current read pointer */
        addr = regReadPair(ERDPTL);

        /* Setup the DMA start pointer */
        regWritePair(EDMASTL, addr);

        /* Add the count to the start address */
        addr += n;
        if(addr > BUF_RX_END) /* RX_END is a valid byte */
        {
            addr -= BUF_RX_END;
        }

        /* Write the DMA end pointer */
        regWritePair(EDMANDL, addr);

        /* Setup the destination address */
        addr = regReadPair(EWRPTL);
        regWritePair(EDMADSTL, addr);

        /* Increment the write pointer */
        addr += n;
        regWritePair(EWRPTL, addr);

        /* CSUMEN is never set so should be clear */

        /* Start DMA and wait for completion */
        regSetBits(ECON1, ECON1_DMAST);
        regWait(ECON1, ECON1_DMAST, 0);
    }
}


void enc28j60TxPktSend(void)
{
    uint16_t pktEnd;

    /* Get the address of the last byte in the packet */
    pktEnd = regReadPair(EWRPTL) - 1;

    /* Set the Tx end pointer */
    regWritePair(ETXNDL, pktEnd);

    /* Setup completion interrupt? */

    /* Start the transmission */
    regSetBits(ECON1, ECON1_TXRTS);
}


/** Move to the start of the next received packet.
 * \retval TRUE  A packet has been received and is ready for reading.
 * \retval FALSE No new packets have been received.
 */
boolean_t enc28j60RxPktStart(void)
{
    /* Check if there are any packets to read */
    if(regRead(EPKTCNT) == 0)
    {
        return FALSE;
    }
    else
    {
        enc28j60RxPktRewind();
        return TRUE;
    }
}


/** Return to the start of some received packet.
 * After calling enc28j60RxPktStart(), this can be used to reset the read
 * pointer to the start of the packet.
 */
void enc28j60RxPktRewind(void)
{
    uint8_t v[6];

    /* Move to the start of the packet and read status vector */
    seekBufPktStart();
    enc28j60RxPktRead(6, v);

    /* Decode the packet byte count, remove CRC count */
    rxPktBytesLength = v[2] | (v[3] << 8);
    rxPktBytesLength -= ETHERNET_CRC_BYTES;

    /* Reset consumed counter */
    rxPktBytesConsumed  = 0;
}


/** Get the current offset in the Rx packet.
 * Returns the count of bytes that have been read, skipped or compared in
 * the current received packet.
 */
uint16_t enc28j60RxPktOffset(void)
{
    return rxPktBytesConsumed;
}


/** Read data from the buffer memory.
 * This reads \a n bytes of data from the buffer memory int \a *data.
 * If \a *data is NULL, data is not read but the read pointer is advanced
 * by \a n bytes.
 *
 * \param[in] n         The number of bytes to read or skip, which maybe zero.
 * \param[in,out] data  Pointer to fill with read data, or NULL if bytes are
 *                       simply to be skipped.
 */
void enc28j60RxPktRead(uint8_t n, void *data)
{
    uint8_t *p = (uint8_t *)data;

    /* Account for the bytes being read */
    rxPktBytesConsumed += n;

    /* Check that there is something to be done */
    while(n > 0)
    {
        const uint8_t b = readOp(0, OP_READ_BUF_MEM);

        if(p != NULL)
        {
            *p = b;
            p++;
        }
        n--;
    }
}


/** Compare some received bytes with a memory buffer.
 * Check whether the next string of bytes in the receive buffer match some
 * pattern.  This always consumes \a n bytes from the receive buffer,
 * independent of what portion of the buffer may have matched.
 *
 * \param[in] n     The count of bytes to process.
 * \param[in] data  Pointer to sought pattern.
 * \retval  TRUE if the next \a n bytes matched data[0] to data[n].
 */
boolean_t enc28j60RxPktCmp(uint8_t n, const void *data)
{
    boolean_t      result = TRUE;
    const uint8_t *p      = data;

    /* Account for the bytes being read */
    rxPktBytesConsumed += n;

    /* Check that there is something to be done */
    while(n > 0)
    {
        const uint8_t b = readOp(0, OP_READ_BUF_MEM);

        if(*p != b) result = FALSE;

        p++;
        n--;
    }

    return result;
}


/** Compare some received bytes with a program memory buffer.
 * This is identical to enc28j60RxPktCmp(), with the exception that \a *data
 * is assumed to be located in program memory.
 *
 * \param[in] n     The count of bytes to process.
 * \param[in] data  Pointer to sought pattern, in program memory.
 * \retval  TRUE if the next \a n bytes matched data[0] to data[n].
 */
boolean_t enc28j60RxPktCmpPrgMem(uint8_t n, const void *data)
{
    boolean_t      result = TRUE;
    const uint8_t *p      = data;

    /* Account for the bytes being read */
    rxPktBytesConsumed += n;

    /* Check that there is something to be done */
    while(n > 0)
    {
        const uint8_t b = readOp(0, OP_READ_BUF_MEM);

        if(pgm_read_byte_near(p) != b) result = FALSE;

        p++;
        n--;
    }

    return result;
}


/** Check if all bytes from the packet have been processed.
 *
 * \retval TRUE  If all bytes in the current packet have been read, skipped,
 *                or compared.
 */
boolean_t enc28j60RxEop(void)
{
    return rxPktBytesConsumed >= rxPktBytesLength;
}


void enc28j60RxPktFree(void)
{
    uint8_t nextPkt[2];

    /* Seek buffer pointer to the start of the packet */
    seekBufPktStart();

    /* Read the address of the next packet */
    enc28j60RxPktRead(2, nextPkt);

    /* Advance the Rx read pointer, freeing the storage */
    regWrite(ERXRDPTL, nextPkt[0]);
    regWrite(ERXRDPTH, nextPkt[1]);

    /* Decrement the packet count */
    regSetBits(ECON2, ECON2_PKTDEC);
}


/** Get the chip revision.
 * This returns the chip revision which can be useful in determining
 * applicable errata.
 *
 * \return The 5-bit chip Id.
 */
uint8_t enc28j60GetRevId(void)
{
    return regRead(EREVID);
}


/** Return whether the link is up or down.
 */
boolean_t enc28j60IsLinkUp(void)
{
    return (phyRead(PHSTAT2) & PHSTAT2_LSTAT) != 0;
}

/** Get the MAC address of this host.
 *
 * \param[in,out] mac  Pointer to be populated with the MAC address.
 */
void enc28j60GetMac(macaddr_t *const mac)
{
    for(uint8_t t = 0; t < sizeof(macaddr_t); t++)
    {
        mac->b[t] = pgm_read_byte_near(&initwords[INIT_MAC_ADDR_START + t].data);
    }
}

/* END OF FILE */

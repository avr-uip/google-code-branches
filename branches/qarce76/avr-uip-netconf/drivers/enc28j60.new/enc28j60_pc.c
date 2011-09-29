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
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <pcap.h>
#include "enc28j60.h"
#include "event.h"
#include "main.h"

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

static uint8_t    txBuffer[1500], rxBuffer[1500];
static uint8_t   *txPtr, *rxPtr;
static uint8_t    rxPacketCount = 0;
static macaddr_t  macAddr = { { 0xc0, 0x01, 0xca, 0xfe, 0xba, 0xbe } };

static pcap_t    *pcapDev = NULL;
static char       pcapErrBuf[PCAP_ERRBUF_SIZE];




static pthread_t       rxThread;
static pthread_cond_t  rxCondition = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t rxMutex     = PTHREAD_MUTEX_INITIALIZER;

/**************************************************************************
 * Local Functions
 **************************************************************************/

/** Check if some packet of data should pass MAC filtering.
 *
 *
 * \retval TRUE  The packet passes and should be processed.
 * \retval FALSE The packet fails and should be dropped.
 */
static boolean_t rxMacFilter(const u_char *data)
{
    /* Check if it is from out MAC address */
    if(memcmp(&data[6], &macAddr.b[0], sizeof(macaddr_t)) == 0)
    {
        /* Drop the packet; we don't normally see our own packets */
        return FALSE;
    }

    /* Check if it is for our MAC address */
    if(memcmp(data, &macAddr.b[0], sizeof(macaddr_t)) == 0)
    {
        return TRUE;
    }

    /* Compare against the broadcast address */
    for(uint8_t t = 0; t < sizeof(macaddr_t); t++)
    {
        if(data[t] != 0xff) return FALSE;
    }

    return TRUE;

}

static void rxLoopHandler(u_char                   *param,
                          const struct pcap_pkthdr *header,
                          const u_char             *pktData)
{
    boolean_t drop = FALSE;

#if defined(PC_SIM_LOSSY_RX)
    if(rand() < (RAND_MAX / 2))
    {
        printf("enc28j60: Rx Packet dropped\n");
        drop = TRUE;
    }
#endif

    pthread_mutex_lock(&rxMutex);

    /* Check if the packet passes the MAC filter */
    if(rxMacFilter(pktData) && !drop)
    {
        /* Copy the packet into the RX buffer */
        memcpy(rxBuffer, pktData, header->len);

        rxPacketCount++;

        M_EventSet(EV_ETH);
    }

    pthread_mutex_unlock(&rxMutex);
}


static void *rxLoop(void *param)
{
    while(1)
    {
        pcap_loop(pcapDev, 1, rxLoopHandler, NULL);

        /* Wait for outstanding packets to be processed */
        pthread_mutex_lock(&rxMutex);
        while(rxPacketCount != 0)
        {
            pthread_cond_wait(&rxCondition, &rxMutex);
        }
        pthread_mutex_unlock(&rxMutex);

    }
}

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
    /* Make the MAC adddress unique */
    macAddr.b[0] = getpid() >>  0;
    macAddr.b[1] = getpid() >>  8;
/*  macAddr.b[2] = getpid() >> 16;
    macAddr.b[3] = getpid() >> 24; */

    /* My Windows interfaces:
     *  VMware 2: \\Device\\NPF_{2FF76462-9A57-4968-BA13-10B54340D1CA}
     *  VMware 8: \\Device\\NPF_{64634012-DD4A-414C-B8B3-6A84E2ED1A3F}
     *  Real NIC: \\Device\\NPF_{E104169E-9DB2-4062-A655-01C622A8CBC0}
     *  Wireless: \\Device\\NPF_{0921DFC0-4B4A-4AC5-9E71-EE5E6ECCBC8B}
     *
     * For Linux, use 'eth0' or appropriate device name.
     */
    pcapDev = pcap_open_live("\\Device\\NPF_{E104169E-9DB2-4062-A655-01C622A8CBC0}",
                             65536,         /* portion of the packet to capture */
                             0,             /* non-zero, promiscuous mode */
                             1,             /* read timeout */
                             pcapErrBuf);
    if(pcapDev == NULL)
    {
        printf("Failed to open packet interface:\n%s\n", pcapErrBuf);
        exit(0);
    }

    pthread_create(&rxThread, NULL, rxLoop, NULL);
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
    static boolean_t linkUp  = FALSE;
    boolean_t        linkNow = enc28j60IsLinkUp();

    if(linkEv && linkUp != linkNow)
    {
        linkUp = linkNow;
        linkEv(linkData);
    }

    if(rxPacketCount != 0)
    {
        if(packetEv != NULL) packetEv(packetData);
    }

}

void enc28j60TxPktStart(void)
{
    txPtr = &txBuffer[0];
}

void enc28j60TxPktWriteMacAddr(macaddrtype_t type)
{
    switch(type)
    {
        case MAC_ADDR_TYP_LOCAL:
            enc28j60TxPktAppend(sizeof(macaddr_t), &macAddr);
            break;

        case MAC_ADDR_TYP_BCAST:
            enc28j60TxPktFill(sizeof(macaddr_t), 0xff);
            break;

        case MAC_ADDR_TYP_ZERO:
            enc28j60TxPktFill(sizeof(macaddr_t), 0x00);
            break;
        default:
            assert(FALSE);
    }
}

void enc28j60TxPktAppend(uint8_t n, const void *data)
{
    memcpy(txPtr, data, n);
    txPtr += n;

    assert(txPtr < &txBuffer[1500]);
}

void enc28j60TxPktFill(uint16_t n, uint8_t pattern)
{
    while(n > 0)
    {
        *txPtr = pattern;
        n--; txPtr++;
    }

    assert(txPtr < &txBuffer[1500]);
}

void enc28j60TxPktAppendRx(uint16_t n)
{
    uint8_t buf[n];

    enc28j60RxPktRead  (n, buf);
    enc28j60TxPktAppend(n, buf);
}

void enc28j60TxPktSend(void)
{
    uint16_t  len  = txPtr - txBuffer;
    boolean_t drop = FALSE;

    assert(pcapDev != NULL);
    assert(len != 0);

#if defined(PC_SIM_LOSSY_TX)
    if(rand() < (RAND_MAX / 2))
    {
        printf("enc28j60: Tx Packet dropped\n");
        drop = TRUE;
    }
#endif

#if defined(ENC28J60_DUMP_TX)
    printf("enc28j60TxPktSend: %u bytes:", len);
    for(uint16_t t = 0; t < len; t++)
    {
        switch(t % 16)
        {
            case 0: printf("\n"); break;
            case 8: printf(" "); break;
        }
        printf("%02X ", txBuffer[t]);
    }
    printf("\n");
#endif

    /* Send down the packet */
    if(!drop)
    {
        if(pcap_sendpacket(pcapDev, txBuffer, len) != 0)
        {
            fprintf(stderr, "Error sending packet: %s\n", pcap_geterr(pcapDev));
        }
    }

    /* Empty the Tx buffer */
    txPtr = &txBuffer[0];
}

boolean_t enc28j60RxPktStart(void)
{
    boolean_t r = FALSE;

    pthread_mutex_lock(&rxMutex);
    if(rxPacketCount > 0)
    {
        rxPtr = &rxBuffer[0];

#if defined(ENC28J60_DUMP_RX)
        printf("enc28j60RxPktStart:");
        for(uint16_t t = 0; t < 80; t++)
        {
            switch(t % 16)
            {
                case 0: printf("\n"); break;
                case 8: printf(" "); break;
            }
            printf("%02X ", rxBuffer[t]);
        }
        printf("\n");
#endif

        r = TRUE;
    }
    pthread_mutex_unlock(&rxMutex);

    return r;
}

void enc28j60RxPktRead(uint8_t n, void *data)
{
    if(data) memcpy(data, rxPtr, n);
    rxPtr += n;

    assert(rxPtr < &rxBuffer[1500]);
}

boolean_t enc28j60RxPktCmp(uint8_t n, const void *data)
{
    boolean_t r;

    r = memcmp(data, rxPtr, n) ? FALSE : TRUE;
    rxPtr += n;

    assert(rxPtr < &rxBuffer[1500]);

    return r;
}

uint16_t enc28j60RxPktOffset(void)
{
    return rxPtr - rxBuffer;
}

boolean_t enc28j60RxEop(void)
{
    return FALSE;
}

void enc28j60RxPktFree(void)
{
    pthread_mutex_lock(&rxMutex);

    rxPacketCount--;

    pthread_cond_signal(&rxCondition);
    pthread_mutex_unlock(&rxMutex);
}

uint8_t enc28j60GetRevId(void)
{
    return 1;
}

boolean_t enc28j60IsLinkUp(void)
{
    return TRUE;
}

void enc28j60GetMac(macaddr_t *const mac)
{
    memcpy(mac, &macAddr, sizeof(macaddr_t));
}

void enc28j60RxPktRewind(void)
{
    rxPtr = &rxBuffer[0];
}

/* END OF FILE */

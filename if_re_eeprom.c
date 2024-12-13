/*
 * Copyright (c) 1997, 1998
 *	Bill Paul <wpaul@ctr.columbia.edu>.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Bill Paul.
 * 4. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Bill Paul AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/dev/if_rl.c,v 1.38.2.7 2001/07/19 18:33:07 wpaul Exp $
 */

/*
 * RealTek 8129/8139 PCI NIC driver
 *
 * Written by Bill Paul <wpaul@ctr.columbia.edu>
 * Electrical Engineering Department
 * Columbia University, New York City
 */

#include <sys/cdefs.h>

#ifdef ENABLE_FIBER_SUPPORT
#define FIBER_SUFFIX "-FIBER"
#else
#define FIBER_SUFFIX ""
#endif
#define RE_VERSION "1.100.00" FIBER_SUFFIX

__FBSDID("$FreeBSD: src/sys/dev/re/if_re.c,v " RE_VERSION __DATE__ " " __TIME__ "  wpaul Exp $");

/*
* This driver also support Realtek RTL8110/RTL8169, RTL8111/RTL8168, RTL8125, RTL8126, and RTL8136/RTL810x.
*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_private.h>	/* until driver is converted */
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/bpf.h>

#include <vm/vm.h>              /* for vtophys */
#include <vm/pmap.h>            /* for vtophys */
#include <machine/clock.h>      /* for DELAY */

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/endian.h>

#include <dev/mii/mii.h>
#include "if_rereg.h"
#ifdef ENABLE_FIBER_SUPPORT
#include <dev/re/if_fiber.h>
#endif //ENABLE_FIBER_SUPPORT

#if OS_VER < VERSION(5,3)
#include <pci/pcireg.h>
#include <pci/pcivar.h>
#include <machine/bus_pio.h>
#include <machine/bus_memio.h>
#else
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <sys/module.h>
#endif

#if OS_VER > VERSION(5,9)
#include <sys/cdefs.h>
#include <sys/endian.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>
#endif

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>

#include <machine/in_cksum.h>

#include "opt_inet.h"
#include "opt_inet6.h"

#include "if_re_eeprom.h"

/*----------------------------------------------------------------------------*/
/*	8139 (CR9346) 9346 command register bits (offset 0x50, 1 byte)*/
/*----------------------------------------------------------------------------*/
#define CR9346_EEDO				0x01			/* 9346 data out*/
#define CR9346_EEDI				0x02			/* 9346 data in*/
#define CR9346_EESK				0x04			/* 9346 serial clock*/
#define CR9346_EECS				0x08			/* 9346 chip select*/
#define CR9346_EEM0				0x40			/* select 8139 operating mode*/
#define CR9346_EEM1				0x80			/* 00: normal*/
#define CR9346_CFGRW			0xC0			/* Config register write*/
#define CR9346_NORM			0x00

/*----------------------------------------------------------------------------*/
/*	EEPROM bit definitions(EEPROM control register bits)*/
/*----------------------------------------------------------------------------*/
#define EN_TRNF					0x10			/* Enable turnoff*/
#define EEDO						CR9346_EEDO	/* EEPROM data out*/
#define EEDI						CR9346_EEDI		/* EEPROM data in (set for writing data)*/
#define EECS						CR9346_EECS		/* EEPROM chip select (1=high, 0=low)*/
#define EESK						CR9346_EESK		/* EEPROM shift clock (1=high, 0=low)*/

/*----------------------------------------------------------------------------*/
/*	EEPROM opcodes*/
/*----------------------------------------------------------------------------*/
#define EEPROM_READ_OPCODE	06
#define EEPROM_WRITE_OPCODE	05
#define EEPROM_ERASE_OPCODE	07
#define EEPROM_EWEN_OPCODE	19				/* Erase/write enable*/
#define EEPROM_EWDS_OPCODE	16				/* Erase/write disable*/

#define	CLOCK_RATE				50				/* us*/

#define RaiseClock(_sc,_x)				\
	(_x) = (_x) | EESK;					\
	CSR_WRITE_1((_sc), RE_EECMD, (_x));	\
	DELAY(CLOCK_RATE);

#define LowerClock(_sc,_x)				\
	(_x) = (_x) & ~EESK;					\
	CSR_WRITE_1((_sc), RE_EECMD, (_x));	\
	DELAY(CLOCK_RATE);

/*
 * Shift out bit(s) to the EEPROM.
 */
void
re_eeprom_ShiftOutBits(struct re_softc *sc, int data, int count)
{
        u_int16_t x, mask;

        mask = 0x01 << (count - 1);
        x = CSR_READ_1(sc, RE_EECMD);

        x &= ~(EEDO | EEDI);

        do {
                x &= ~EEDI;
                if (data & mask)
                        x |= EEDI;

                CSR_WRITE_1(sc, RE_EECMD, x);
                DELAY(CLOCK_RATE);
                RaiseClock(sc,x);
                LowerClock(sc,x);
                mask = mask >> 1;
        } while (mask);

        x &= ~EEDI;
        CSR_WRITE_1(sc, RE_EECMD, x);
}

/*
 * Shift in bit(s) from the EEPROM.
 */
static u_int16_t re_eeprom_ShiftInBits(struct re_softc *sc)
{
        u_int16_t x,d,i;
        x = CSR_READ_1(sc, RE_EECMD);

        x &= ~(EEDO | EEDI);
        d = 0;

        for (i=0; i<16; i++) {
                d = d << 1;
                RaiseClock(sc, x);

                x = CSR_READ_1(sc, RE_EECMD);

                x &= ~(EEDI);
                if (x & EEDO)
                        d |= 1;

                LowerClock(sc, x);
        }

        return d;
}

/*
 * Clean up EEprom read/write setting
 */
static void re_eeprom_EEpromCleanup(struct re_softc *sc)
{
        u_int16_t x;
        x = CSR_READ_1(sc, RE_EECMD);

        x &= ~(EECS | EEDI);
        CSR_WRITE_1(sc, RE_EECMD, x);

        RaiseClock(sc, x);
        LowerClock(sc, x);
}

/*
 * Read a word of data stored in the EEPROM at address 'addr.'
 */
static void re_eeprom_getword(struct re_softc *sc, int addr, u_int16_t *dest)
{
        u_int16_t x;

        /* select EEPROM, reset bits, set EECS*/
        x = CSR_READ_1(sc, RE_EECMD);

        x &= ~(EEDI | EEDO | EESK | CR9346_EEM0);
        x |= CR9346_EEM1 | EECS;
        CSR_WRITE_1(sc, RE_EECMD, x);

        /* write the read opcode and register number in that order*/
        /* The opcode is 3bits in length, reg is 6 bits long*/
        re_eeprom_ShiftOutBits(sc, EEPROM_READ_OPCODE, 3);

        if (CSR_READ_4(sc, RE_RXCFG) & RE_RXCFG_RX_9356SEL)
                re_eeprom_ShiftOutBits(sc, addr,8);	/*93c56=8*/
        else
                re_eeprom_ShiftOutBits(sc, addr,6);	/*93c46=6*/

        /* Now read the data (16 bits) in from the selected EEPROM word*/
        *dest=re_eeprom_ShiftInBits(sc);

        re_eeprom_EEpromCleanup(sc);
        return;
}

/*
 * Read a sequence of words from the EEPROM.
 */
void
re_read_eeprom(struct re_softc *sc, caddr_t dest, int off, int cnt,
    int swap)
{
        int			i;
        u_int16_t		word = 0, *ptr;

        for (i = 0; i < cnt; i++) {
                re_eeprom_getword(sc, off + i, &word);
                ptr = (u_int16_t *)(dest + (i * 2));
                if (swap)
                        *ptr = ntohs(word);
                else
                        *ptr = word;
        }

        return;
}

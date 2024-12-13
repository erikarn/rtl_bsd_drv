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
#include "if_re_mdio.h"
#include "if_re_eri.h"
#include "if_re_efuse.h"
#include "if_re_ocp.h"
#include "if_re_cfg.h"
#include "if_re_csi.h"
#include "if_re_mac_mcu.h"
#include "if_re_phy_mcu.h"

#include "if_re_phy_8411.h"

void
re_set_phy_mcu_8411_1(struct re_softc *sc)
{
        u_int16_t PhyRegValue;
        int i;

        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x00, 0x1800);
        PhyRegValue = re_mdio_read(sc, 0x15);
        PhyRegValue &= ~(BIT_12);
        re_mdio_write(sc, 0x15, PhyRegValue);
        re_mdio_write(sc, 0x00, 0x4800);
        re_mdio_write(sc, 0x1f, 0x0007);
        re_mdio_write(sc, 0x1e, 0x002f);
        for (i = 0; i < 1000; i++) {
                DELAY(100);
                PhyRegValue = re_mdio_read(sc, 0x1c);
                if (PhyRegValue & BIT_7)
                        break;
        }
        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x00, 0x1800);
        re_mdio_write(sc, 0x1f, 0x0007);
        re_mdio_write(sc, 0x1e, 0x0023);
        for (i = 0; i < 200; i++) {
                DELAY(100);
                PhyRegValue = re_mdio_read(sc, 0x18);
                if (!(PhyRegValue & BIT_0))
                        break;
        }
        re_mdio_write(sc, 0x1f, 0x0005);
        re_mdio_write(sc, 0x05, 0xfff6);
        re_mdio_write(sc, 0x06, 0x0080);
        re_mdio_write(sc, 0x1f, 0x0007);
        re_mdio_write(sc, 0x1e, 0x0023);
        re_mdio_write(sc, 0x16, 0x0306);
        re_mdio_write(sc, 0x16, 0x0307);
        re_mdio_write(sc, 0x15, 0x0098);
        re_mdio_write(sc, 0x19, 0x7c0b);
        re_mdio_write(sc, 0x15, 0x0099);
        re_mdio_write(sc, 0x19, 0x6c0b);
        re_mdio_write(sc, 0x15, 0x00eb);
        re_mdio_write(sc, 0x19, 0x6c0b);
        re_mdio_write(sc, 0x15, 0x00f8);
        re_mdio_write(sc, 0x19, 0x6f0b);
        re_mdio_write(sc, 0x15, 0x00fe);
        re_mdio_write(sc, 0x19, 0x6f0f);
        re_mdio_write(sc, 0x15, 0x00db);
        re_mdio_write(sc, 0x19, 0x6f09);
        re_mdio_write(sc, 0x15, 0x00dc);
        re_mdio_write(sc, 0x19, 0xaefd);
        re_mdio_write(sc, 0x15, 0x00dd);
        re_mdio_write(sc, 0x19, 0x6f0b);
        re_mdio_write(sc, 0x15, 0x00de);
        re_mdio_write(sc, 0x19, 0xc60b);
        re_mdio_write(sc, 0x15, 0x00df);
        re_mdio_write(sc, 0x19, 0x00fa);
        re_mdio_write(sc, 0x15, 0x00e0);
        re_mdio_write(sc, 0x19, 0x30e1);
        re_mdio_write(sc, 0x15, 0x020c);
        re_mdio_write(sc, 0x19, 0x3224);
        re_mdio_write(sc, 0x15, 0x020e);
        re_mdio_write(sc, 0x19, 0x9813);
        re_mdio_write(sc, 0x15, 0x020f);
        re_mdio_write(sc, 0x19, 0x7801);
        re_mdio_write(sc, 0x15, 0x0210);
        re_mdio_write(sc, 0x19, 0x930f);
        re_mdio_write(sc, 0x15, 0x0211);
        re_mdio_write(sc, 0x19, 0x9206);
        re_mdio_write(sc, 0x15, 0x0212);
        re_mdio_write(sc, 0x19, 0x4002);
        re_mdio_write(sc, 0x15, 0x0213);
        re_mdio_write(sc, 0x19, 0x7800);
        re_mdio_write(sc, 0x15, 0x0214);
        re_mdio_write(sc, 0x19, 0x588f);
        re_mdio_write(sc, 0x15, 0x0215);
        re_mdio_write(sc, 0x19, 0x5520);
        re_mdio_write(sc, 0x15, 0x0216);
        re_mdio_write(sc, 0x19, 0x3224);
        re_mdio_write(sc, 0x15, 0x0217);
        re_mdio_write(sc, 0x19, 0x4002);
        re_mdio_write(sc, 0x15, 0x0218);
        re_mdio_write(sc, 0x19, 0x7800);
        re_mdio_write(sc, 0x15, 0x0219);
        re_mdio_write(sc, 0x19, 0x588d);
        re_mdio_write(sc, 0x15, 0x021a);
        re_mdio_write(sc, 0x19, 0x5540);
        re_mdio_write(sc, 0x15, 0x021b);
        re_mdio_write(sc, 0x19, 0x9e03);
        re_mdio_write(sc, 0x15, 0x021c);
        re_mdio_write(sc, 0x19, 0x7c40);
        re_mdio_write(sc, 0x15, 0x021d);
        re_mdio_write(sc, 0x19, 0x6840);
        re_mdio_write(sc, 0x15, 0x021e);
        re_mdio_write(sc, 0x19, 0x3224);
        re_mdio_write(sc, 0x15, 0x021f);
        re_mdio_write(sc, 0x19, 0x4002);
        re_mdio_write(sc, 0x15, 0x0220);
        re_mdio_write(sc, 0x19, 0x3224);
        re_mdio_write(sc, 0x15, 0x0221);
        re_mdio_write(sc, 0x19, 0x9e03);
        re_mdio_write(sc, 0x15, 0x0222);
        re_mdio_write(sc, 0x19, 0x7c40);
        re_mdio_write(sc, 0x15, 0x0223);
        re_mdio_write(sc, 0x19, 0x6840);
        re_mdio_write(sc, 0x15, 0x0224);
        re_mdio_write(sc, 0x19, 0x7800);
        re_mdio_write(sc, 0x15, 0x0225);
        re_mdio_write(sc, 0x19, 0x3231);
        re_mdio_write(sc, 0x15, 0x0000);
        re_mdio_write(sc, 0x16, 0x0306);
        re_mdio_write(sc, 0x16, 0x0300);
        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x1f, 0x0005);
        re_mdio_write(sc, 0x05, 0xfff6);
        re_mdio_write(sc, 0x06, 0x0080);
        re_mdio_write(sc, 0x05, 0x8000);
        re_mdio_write(sc, 0x06, 0x0280);
        re_mdio_write(sc, 0x06, 0x48f7);
        re_mdio_write(sc, 0x06, 0x00e0);
        re_mdio_write(sc, 0x06, 0xfff7);
        re_mdio_write(sc, 0x06, 0xa080);
        re_mdio_write(sc, 0x06, 0x02ae);
        re_mdio_write(sc, 0x06, 0xf602);
        re_mdio_write(sc, 0x06, 0x011e);
        re_mdio_write(sc, 0x06, 0x0201);
        re_mdio_write(sc, 0x06, 0x2b02);
        re_mdio_write(sc, 0x06, 0x8077);
        re_mdio_write(sc, 0x06, 0x0201);
        re_mdio_write(sc, 0x06, 0x4802);
        re_mdio_write(sc, 0x06, 0x0162);
        re_mdio_write(sc, 0x06, 0x0280);
        re_mdio_write(sc, 0x06, 0x9402);
        re_mdio_write(sc, 0x06, 0x810e);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x88e1);
        re_mdio_write(sc, 0x06, 0x8b89);
        re_mdio_write(sc, 0x06, 0x1e01);
        re_mdio_write(sc, 0x06, 0xe18b);
        re_mdio_write(sc, 0x06, 0x8a1e);
        re_mdio_write(sc, 0x06, 0x01e1);
        re_mdio_write(sc, 0x06, 0x8b8b);
        re_mdio_write(sc, 0x06, 0x1e01);
        re_mdio_write(sc, 0x06, 0xe18b);
        re_mdio_write(sc, 0x06, 0x8c1e);
        re_mdio_write(sc, 0x06, 0x01e1);
        re_mdio_write(sc, 0x06, 0x8b8d);
        re_mdio_write(sc, 0x06, 0x1e01);
        re_mdio_write(sc, 0x06, 0xe18b);
        re_mdio_write(sc, 0x06, 0x8e1e);
        re_mdio_write(sc, 0x06, 0x01a0);
        re_mdio_write(sc, 0x06, 0x00c7);
        re_mdio_write(sc, 0x06, 0xaebb);
        re_mdio_write(sc, 0x06, 0xd481);
        re_mdio_write(sc, 0x06, 0xd4e4);
        re_mdio_write(sc, 0x06, 0x8b92);
        re_mdio_write(sc, 0x06, 0xe58b);
        re_mdio_write(sc, 0x06, 0x9302);
        re_mdio_write(sc, 0x06, 0x2e5a);
        re_mdio_write(sc, 0x06, 0xbf8b);
        re_mdio_write(sc, 0x06, 0x88ec);
        re_mdio_write(sc, 0x06, 0x0019);
        re_mdio_write(sc, 0x06, 0xa98b);
        re_mdio_write(sc, 0x06, 0x90f9);
        re_mdio_write(sc, 0x06, 0xeeff);
        re_mdio_write(sc, 0x06, 0xf600);
        re_mdio_write(sc, 0x06, 0xeeff);
        re_mdio_write(sc, 0x06, 0xf7fc);
        re_mdio_write(sc, 0x06, 0xd100);
        re_mdio_write(sc, 0x06, 0xbf83);
        re_mdio_write(sc, 0x06, 0x3c02);
        re_mdio_write(sc, 0x06, 0x3a21);
        re_mdio_write(sc, 0x06, 0xd101);
        re_mdio_write(sc, 0x06, 0xbf83);
        re_mdio_write(sc, 0x06, 0x3f02);
        re_mdio_write(sc, 0x06, 0x3a21);
        re_mdio_write(sc, 0x06, 0x04f8);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x8aad);
        re_mdio_write(sc, 0x06, 0x2014);
        re_mdio_write(sc, 0x06, 0xee8b);
        re_mdio_write(sc, 0x06, 0x8a00);
        re_mdio_write(sc, 0x06, 0x0220);
        re_mdio_write(sc, 0x06, 0x8be0);
        re_mdio_write(sc, 0x06, 0xe426);
        re_mdio_write(sc, 0x06, 0xe1e4);
        re_mdio_write(sc, 0x06, 0x27ee);
        re_mdio_write(sc, 0x06, 0xe426);
        re_mdio_write(sc, 0x06, 0x23e5);
        re_mdio_write(sc, 0x06, 0xe427);
        re_mdio_write(sc, 0x06, 0xfc04);
        re_mdio_write(sc, 0x06, 0xf8e0);
        re_mdio_write(sc, 0x06, 0x8b8d);
        re_mdio_write(sc, 0x06, 0xad20);
        re_mdio_write(sc, 0x06, 0x14ee);
        re_mdio_write(sc, 0x06, 0x8b8d);
        re_mdio_write(sc, 0x06, 0x00e0);
        re_mdio_write(sc, 0x06, 0x8a5a);
        re_mdio_write(sc, 0x06, 0x7803);
        re_mdio_write(sc, 0x06, 0x9e09);
        re_mdio_write(sc, 0x06, 0x0206);
        re_mdio_write(sc, 0x06, 0x2802);
        re_mdio_write(sc, 0x06, 0x80b1);
        re_mdio_write(sc, 0x06, 0x0232);
        re_mdio_write(sc, 0x06, 0xfdfc);
        re_mdio_write(sc, 0x06, 0x04f8);
        re_mdio_write(sc, 0x06, 0xf9e0);
        re_mdio_write(sc, 0x06, 0x8b81);
        re_mdio_write(sc, 0x06, 0xac26);
        re_mdio_write(sc, 0x06, 0x1ae0);
        re_mdio_write(sc, 0x06, 0x8b81);
        re_mdio_write(sc, 0x06, 0xac21);
        re_mdio_write(sc, 0x06, 0x14e0);
        re_mdio_write(sc, 0x06, 0x8b85);
        re_mdio_write(sc, 0x06, 0xac20);
        re_mdio_write(sc, 0x06, 0x0ee0);
        re_mdio_write(sc, 0x06, 0x8b85);
        re_mdio_write(sc, 0x06, 0xac23);
        re_mdio_write(sc, 0x06, 0x08e0);
        re_mdio_write(sc, 0x06, 0x8b87);
        re_mdio_write(sc, 0x06, 0xac24);
        re_mdio_write(sc, 0x06, 0x02ae);
        re_mdio_write(sc, 0x06, 0x3802);
        re_mdio_write(sc, 0x06, 0x1b02);
        re_mdio_write(sc, 0x06, 0xeee4);
        re_mdio_write(sc, 0x06, 0x1c04);
        re_mdio_write(sc, 0x06, 0xeee4);
        re_mdio_write(sc, 0x06, 0x1d04);
        re_mdio_write(sc, 0x06, 0xe2e0);
        re_mdio_write(sc, 0x06, 0x7ce3);
        re_mdio_write(sc, 0x06, 0xe07d);
        re_mdio_write(sc, 0x06, 0xe0e0);
        re_mdio_write(sc, 0x06, 0x38e1);
        re_mdio_write(sc, 0x06, 0xe039);
        re_mdio_write(sc, 0x06, 0xad2e);
        re_mdio_write(sc, 0x06, 0x1bad);
        re_mdio_write(sc, 0x06, 0x390d);
        re_mdio_write(sc, 0x06, 0xd101);
        re_mdio_write(sc, 0x06, 0xbf22);
        re_mdio_write(sc, 0x06, 0xe802);
        re_mdio_write(sc, 0x06, 0x3a21);
        re_mdio_write(sc, 0x06, 0x0222);
        re_mdio_write(sc, 0x06, 0x10ae);
        re_mdio_write(sc, 0x06, 0x0bac);
        re_mdio_write(sc, 0x06, 0x3802);
        re_mdio_write(sc, 0x06, 0xae06);
        re_mdio_write(sc, 0x06, 0x0222);
        re_mdio_write(sc, 0x06, 0x4d02);
        re_mdio_write(sc, 0x06, 0x2292);
        re_mdio_write(sc, 0x06, 0x021b);
        re_mdio_write(sc, 0x06, 0x13fd);
        re_mdio_write(sc, 0x06, 0xfc04);
        re_mdio_write(sc, 0x06, 0xf8e0);
        re_mdio_write(sc, 0x06, 0x8b8e);
        re_mdio_write(sc, 0x06, 0xad20);
        re_mdio_write(sc, 0x06, 0x1af6);
        re_mdio_write(sc, 0x06, 0x20e4);
        re_mdio_write(sc, 0x06, 0x8b8e);
        re_mdio_write(sc, 0x06, 0x022b);
        re_mdio_write(sc, 0x06, 0x1e02);
        re_mdio_write(sc, 0x06, 0x82ae);
        re_mdio_write(sc, 0x06, 0x0203);
        re_mdio_write(sc, 0x06, 0xc002);
        re_mdio_write(sc, 0x06, 0x827d);
        re_mdio_write(sc, 0x06, 0x022e);
        re_mdio_write(sc, 0x06, 0x6f02);
        re_mdio_write(sc, 0x06, 0x047b);
        re_mdio_write(sc, 0x06, 0x022f);
        re_mdio_write(sc, 0x06, 0x9ae0);
        re_mdio_write(sc, 0x06, 0x8b8e);
        re_mdio_write(sc, 0x06, 0xad21);
        re_mdio_write(sc, 0x06, 0x0bf6);
        re_mdio_write(sc, 0x06, 0x21e4);
        re_mdio_write(sc, 0x06, 0x8b8e);
        re_mdio_write(sc, 0x06, 0x0281);
        re_mdio_write(sc, 0x06, 0x9002);
        re_mdio_write(sc, 0x06, 0x1cd9);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x8ead);
        re_mdio_write(sc, 0x06, 0x2208);
        re_mdio_write(sc, 0x06, 0xf622);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x8e02);
        re_mdio_write(sc, 0x06, 0x35f4);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x8ead);
        re_mdio_write(sc, 0x06, 0x2308);
        re_mdio_write(sc, 0x06, 0xf623);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x8e02);
        re_mdio_write(sc, 0x06, 0x31e8);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x8ead);
        re_mdio_write(sc, 0x06, 0x2405);
        re_mdio_write(sc, 0x06, 0xf624);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x8ee0);
        re_mdio_write(sc, 0x06, 0x8b8e);
        re_mdio_write(sc, 0x06, 0xad25);
        re_mdio_write(sc, 0x06, 0x05f6);
        re_mdio_write(sc, 0x06, 0x25e4);
        re_mdio_write(sc, 0x06, 0x8b8e);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x8ead);
        re_mdio_write(sc, 0x06, 0x2608);
        re_mdio_write(sc, 0x06, 0xf626);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x8e02);
        re_mdio_write(sc, 0x06, 0x2d8a);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x8ead);
        re_mdio_write(sc, 0x06, 0x2705);
        re_mdio_write(sc, 0x06, 0xf627);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x8e02);
        re_mdio_write(sc, 0x06, 0x0386);
        re_mdio_write(sc, 0x06, 0xfc04);
        re_mdio_write(sc, 0x06, 0xf8fa);
        re_mdio_write(sc, 0x06, 0xef69);
        re_mdio_write(sc, 0x06, 0xe0e0);
        re_mdio_write(sc, 0x06, 0x00e1);
        re_mdio_write(sc, 0x06, 0xe001);
        re_mdio_write(sc, 0x06, 0xad27);
        re_mdio_write(sc, 0x06, 0x32e0);
        re_mdio_write(sc, 0x06, 0x8b40);
        re_mdio_write(sc, 0x06, 0xf720);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x40bf);
        re_mdio_write(sc, 0x06, 0x32c1);
        re_mdio_write(sc, 0x06, 0x0239);
        re_mdio_write(sc, 0x06, 0xf4ad);
        re_mdio_write(sc, 0x06, 0x2821);
        re_mdio_write(sc, 0x06, 0xe0e0);
        re_mdio_write(sc, 0x06, 0x20e1);
        re_mdio_write(sc, 0x06, 0xe021);
        re_mdio_write(sc, 0x06, 0xad20);
        re_mdio_write(sc, 0x06, 0x18e0);
        re_mdio_write(sc, 0x06, 0x8b40);
        re_mdio_write(sc, 0x06, 0xf620);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x40ee);
        re_mdio_write(sc, 0x06, 0x8b3b);
        re_mdio_write(sc, 0x06, 0xffe0);
        re_mdio_write(sc, 0x06, 0x8a8a);
        re_mdio_write(sc, 0x06, 0xe18a);
        re_mdio_write(sc, 0x06, 0x8be4);
        re_mdio_write(sc, 0x06, 0xe000);
        re_mdio_write(sc, 0x06, 0xe5e0);
        re_mdio_write(sc, 0x06, 0x01ef);
        re_mdio_write(sc, 0x06, 0x96fe);
        re_mdio_write(sc, 0x06, 0xfc04);
        re_mdio_write(sc, 0x06, 0xf8f9);
        re_mdio_write(sc, 0x06, 0xface);
        re_mdio_write(sc, 0x06, 0xfaef);
        re_mdio_write(sc, 0x06, 0x69fa);
        re_mdio_write(sc, 0x06, 0xd401);
        re_mdio_write(sc, 0x06, 0x55b4);
        re_mdio_write(sc, 0x06, 0xfebf);
        re_mdio_write(sc, 0x06, 0x1c5e);
        re_mdio_write(sc, 0x06, 0x0239);
        re_mdio_write(sc, 0x06, 0xf4ac);
        re_mdio_write(sc, 0x06, 0x280b);
        re_mdio_write(sc, 0x06, 0xbf1c);
        re_mdio_write(sc, 0x06, 0x5b02);
        re_mdio_write(sc, 0x06, 0x39f4);
        re_mdio_write(sc, 0x06, 0xac28);
        re_mdio_write(sc, 0x06, 0x49ae);
        re_mdio_write(sc, 0x06, 0x64bf);
        re_mdio_write(sc, 0x06, 0x1c5b);
        re_mdio_write(sc, 0x06, 0x0239);
        re_mdio_write(sc, 0x06, 0xf4ac);
        re_mdio_write(sc, 0x06, 0x285b);
        re_mdio_write(sc, 0x06, 0xd000);
        re_mdio_write(sc, 0x06, 0x0282);
        re_mdio_write(sc, 0x06, 0x62ac);
        re_mdio_write(sc, 0x06, 0x2105);
        re_mdio_write(sc, 0x06, 0xac22);
        re_mdio_write(sc, 0x06, 0x02ae);
        re_mdio_write(sc, 0x06, 0x4ebf);
        re_mdio_write(sc, 0x06, 0xe0c4);
        re_mdio_write(sc, 0x06, 0xbe85);
        re_mdio_write(sc, 0x06, 0xecd2);
        re_mdio_write(sc, 0x06, 0x04d8);
        re_mdio_write(sc, 0x06, 0x19d9);
        re_mdio_write(sc, 0x06, 0x1907);
        re_mdio_write(sc, 0x06, 0xdc19);
        re_mdio_write(sc, 0x06, 0xdd19);
        re_mdio_write(sc, 0x06, 0x0789);
        re_mdio_write(sc, 0x06, 0x89ef);
        re_mdio_write(sc, 0x06, 0x645e);
        re_mdio_write(sc, 0x06, 0x07ff);
        re_mdio_write(sc, 0x06, 0x0d65);
        re_mdio_write(sc, 0x06, 0x5cf8);
        re_mdio_write(sc, 0x06, 0x001e);
        re_mdio_write(sc, 0x06, 0x46dc);
        re_mdio_write(sc, 0x06, 0x19dd);
        re_mdio_write(sc, 0x06, 0x19b2);
        re_mdio_write(sc, 0x06, 0xe2d4);
        re_mdio_write(sc, 0x06, 0x0001);
        re_mdio_write(sc, 0x06, 0xbf1c);
        re_mdio_write(sc, 0x06, 0x5b02);
        re_mdio_write(sc, 0x06, 0x3a21);
        re_mdio_write(sc, 0x06, 0xae1d);
        re_mdio_write(sc, 0x06, 0xbee0);
        re_mdio_write(sc, 0x06, 0xc4bf);
        re_mdio_write(sc, 0x06, 0x85ec);
        re_mdio_write(sc, 0x06, 0xd204);
        re_mdio_write(sc, 0x06, 0xd819);
        re_mdio_write(sc, 0x06, 0xd919);
        re_mdio_write(sc, 0x06, 0x07dc);
        re_mdio_write(sc, 0x06, 0x19dd);
        re_mdio_write(sc, 0x06, 0x1907);
        re_mdio_write(sc, 0x06, 0xb2f4);
        re_mdio_write(sc, 0x06, 0xd400);
        re_mdio_write(sc, 0x06, 0x00bf);
        re_mdio_write(sc, 0x06, 0x1c5b);
        re_mdio_write(sc, 0x06, 0x023a);
        re_mdio_write(sc, 0x06, 0x21fe);
        re_mdio_write(sc, 0x06, 0xef96);
        re_mdio_write(sc, 0x06, 0xfec6);
        re_mdio_write(sc, 0x06, 0xfefd);
        re_mdio_write(sc, 0x06, 0xfc05);
        re_mdio_write(sc, 0x06, 0xf9e2);
        re_mdio_write(sc, 0x06, 0xe0ea);
        re_mdio_write(sc, 0x06, 0xe3e0);
        re_mdio_write(sc, 0x06, 0xeb5a);
        re_mdio_write(sc, 0x06, 0x070c);
        re_mdio_write(sc, 0x06, 0x031e);
        re_mdio_write(sc, 0x06, 0x20e6);
        re_mdio_write(sc, 0x06, 0xe0ea);
        re_mdio_write(sc, 0x06, 0xe7e0);
        re_mdio_write(sc, 0x06, 0xebe0);
        re_mdio_write(sc, 0x06, 0xe0fc);
        re_mdio_write(sc, 0x06, 0xe1e0);
        re_mdio_write(sc, 0x06, 0xfdfd);
        re_mdio_write(sc, 0x06, 0x04f8);
        re_mdio_write(sc, 0x06, 0xfaef);
        re_mdio_write(sc, 0x06, 0x69e0);
        re_mdio_write(sc, 0x06, 0x8b80);
        re_mdio_write(sc, 0x06, 0xad27);
        re_mdio_write(sc, 0x06, 0x22bf);
        re_mdio_write(sc, 0x06, 0x47ba);
        re_mdio_write(sc, 0x06, 0x0239);
        re_mdio_write(sc, 0x06, 0xf4e0);
        re_mdio_write(sc, 0x06, 0x8b44);
        re_mdio_write(sc, 0x06, 0x1f01);
        re_mdio_write(sc, 0x06, 0x9e15);
        re_mdio_write(sc, 0x06, 0xe58b);
        re_mdio_write(sc, 0x06, 0x44ad);
        re_mdio_write(sc, 0x06, 0x2907);
        re_mdio_write(sc, 0x06, 0xac28);
        re_mdio_write(sc, 0x06, 0x04d1);
        re_mdio_write(sc, 0x06, 0x01ae);
        re_mdio_write(sc, 0x06, 0x02d1);
        re_mdio_write(sc, 0x06, 0x00bf);
        re_mdio_write(sc, 0x06, 0x8342);
        re_mdio_write(sc, 0x06, 0x023a);
        re_mdio_write(sc, 0x06, 0x21ef);
        re_mdio_write(sc, 0x06, 0x96fe);
        re_mdio_write(sc, 0x06, 0xfc04);
        re_mdio_write(sc, 0x06, 0xf8e0);
        re_mdio_write(sc, 0x06, 0x8b85);
        re_mdio_write(sc, 0x06, 0xad26);
        re_mdio_write(sc, 0x06, 0x30e0);
        re_mdio_write(sc, 0x06, 0xe036);
        re_mdio_write(sc, 0x06, 0xe1e0);
        re_mdio_write(sc, 0x06, 0x37e1);
        re_mdio_write(sc, 0x06, 0x8b3f);
        re_mdio_write(sc, 0x06, 0x1f10);
        re_mdio_write(sc, 0x06, 0x9e23);
        re_mdio_write(sc, 0x06, 0xe48b);
        re_mdio_write(sc, 0x06, 0x3fac);
        re_mdio_write(sc, 0x06, 0x200b);
        re_mdio_write(sc, 0x06, 0xac21);
        re_mdio_write(sc, 0x06, 0x0dac);
        re_mdio_write(sc, 0x06, 0x250f);
        re_mdio_write(sc, 0x06, 0xac27);
        re_mdio_write(sc, 0x06, 0x11ae);
        re_mdio_write(sc, 0x06, 0x1202);
        re_mdio_write(sc, 0x06, 0x2cb5);
        re_mdio_write(sc, 0x06, 0xae0d);
        re_mdio_write(sc, 0x06, 0x0282);
        re_mdio_write(sc, 0x06, 0xe7ae);
        re_mdio_write(sc, 0x06, 0x0802);
        re_mdio_write(sc, 0x06, 0x2cd7);
        re_mdio_write(sc, 0x06, 0xae03);
        re_mdio_write(sc, 0x06, 0x022c);
        re_mdio_write(sc, 0x06, 0xeafc);
        re_mdio_write(sc, 0x06, 0x04f8);
        re_mdio_write(sc, 0x06, 0xfaef);
        re_mdio_write(sc, 0x06, 0x6902);
        re_mdio_write(sc, 0x06, 0x8304);
        re_mdio_write(sc, 0x06, 0xe0e0);
        re_mdio_write(sc, 0x06, 0x14e1);
        re_mdio_write(sc, 0x06, 0xe015);
        re_mdio_write(sc, 0x06, 0xad26);
        re_mdio_write(sc, 0x06, 0x08d1);
        re_mdio_write(sc, 0x06, 0x1ebf);
        re_mdio_write(sc, 0x06, 0x2d47);
        re_mdio_write(sc, 0x06, 0x023a);
        re_mdio_write(sc, 0x06, 0x21ef);
        re_mdio_write(sc, 0x06, 0x96fe);
        re_mdio_write(sc, 0x06, 0xfc04);
        re_mdio_write(sc, 0x06, 0xf8e0);
        re_mdio_write(sc, 0x06, 0x8b85);
        re_mdio_write(sc, 0x06, 0xad27);
        re_mdio_write(sc, 0x06, 0x2fd0);
        re_mdio_write(sc, 0x06, 0x0b02);
        re_mdio_write(sc, 0x06, 0x3826);
        re_mdio_write(sc, 0x06, 0x5882);
        re_mdio_write(sc, 0x06, 0x7882);
        re_mdio_write(sc, 0x06, 0x9f24);
        re_mdio_write(sc, 0x06, 0xe08b);
        re_mdio_write(sc, 0x06, 0x32e1);
        re_mdio_write(sc, 0x06, 0x8b33);
        re_mdio_write(sc, 0x06, 0x1f10);
        re_mdio_write(sc, 0x06, 0x9e1a);
        re_mdio_write(sc, 0x06, 0x10e4);
        re_mdio_write(sc, 0x06, 0x8b32);
        re_mdio_write(sc, 0x06, 0xe0e0);
        re_mdio_write(sc, 0x06, 0x28e1);
        re_mdio_write(sc, 0x06, 0xe029);
        re_mdio_write(sc, 0x06, 0xf72c);
        re_mdio_write(sc, 0x06, 0xe4e0);
        re_mdio_write(sc, 0x06, 0x28e5);
        re_mdio_write(sc, 0x06, 0xe029);
        re_mdio_write(sc, 0x06, 0xf62c);
        re_mdio_write(sc, 0x06, 0xe4e0);
        re_mdio_write(sc, 0x06, 0x28e5);
        re_mdio_write(sc, 0x06, 0xe029);
        re_mdio_write(sc, 0x06, 0xfc04);
        re_mdio_write(sc, 0x06, 0x00e1);
        re_mdio_write(sc, 0x06, 0x4077);
        re_mdio_write(sc, 0x06, 0xe140);
        re_mdio_write(sc, 0x06, 0xbbe0);
        re_mdio_write(sc, 0x06, 0x2a00);
        re_mdio_write(sc, 0x05, 0xe142);
        PhyRegValue = re_mdio_read(sc, 0x06);
        PhyRegValue |= BIT_0;
        re_mdio_write(sc, 0x06,PhyRegValue);
        re_mdio_write(sc, 0x05, 0xe140);
        PhyRegValue = re_mdio_read(sc, 0x06);
        PhyRegValue |= BIT_0;
        re_mdio_write(sc, 0x06, PhyRegValue);
        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
                DELAY(100);
                PhyRegValue = re_mdio_read(sc, 0x00);
                if (PhyRegValue & BIT_7)
                        break;
        }
        re_mdio_write(sc, 0x1f, 0x0007);
        re_mdio_write(sc, 0x1e, 0x0023);
        PhyRegValue = re_mdio_read(sc, 0x17);
        PhyRegValue |= BIT_1;
        if (sc->RequiredSecLanDonglePatch)
                PhyRegValue &= ~(BIT_2);
        re_mdio_write(sc, 0x17, PhyRegValue);
        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x1f, 0x0003);
        re_mdio_write(sc, 0x09, 0xA20F);
        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x1f, 0x0003);
        re_mdio_write(sc, 0x01, 0x328A);
        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x1f, 0x0003);
        PhyRegValue = re_mdio_read(sc, 0x19);
        PhyRegValue &= ~BIT_0;
        re_mdio_write(sc, 0x19, PhyRegValue);
        PhyRegValue = re_mdio_read(sc, 0x10);
        PhyRegValue &= ~BIT_10;
        re_mdio_write(sc, 0x10, PhyRegValue);
        re_mdio_write(sc, 0x1f, 0x0000);
        re_mdio_write(sc, 0x00, 0x9200);
}

void
re_set_phy_mcu_8411b_1(struct re_softc *sc)
{
        u_int16_t PhyRegValue;

        re_set_phy_mcu_patch_request(sc);

        re_mdio_write(sc, 0x1f, 0x0A43);
        re_mdio_write(sc, 0x13, 0x8146);
        re_mdio_write(sc, 0x14, 0x0100);
        re_mdio_write(sc, 0x13, 0xB82E);
        re_mdio_write(sc, 0x14, 0x0001);

        re_mdio_write(sc, 0x1F, 0x0A43);
        re_mdio_write(sc, 0x13, 0xb820);
        re_mdio_write(sc, 0x14, 0x0290);
        re_mdio_write(sc, 0x13, 0xa012);
        re_mdio_write(sc, 0x14, 0x0000);
        re_mdio_write(sc, 0x13, 0xa014);
        re_mdio_write(sc, 0x14, 0x2c04);
        re_mdio_write(sc, 0x14, 0x2c07);
        re_mdio_write(sc, 0x14, 0x2c07);
        re_mdio_write(sc, 0x14, 0x2c07);
        re_mdio_write(sc, 0x14, 0xa304);
        re_mdio_write(sc, 0x14, 0xa301);
        re_mdio_write(sc, 0x14, 0x207e);
        re_mdio_write(sc, 0x13, 0xa01a);
        re_mdio_write(sc, 0x14, 0x0000);
        re_mdio_write(sc, 0x13, 0xa006);
        re_mdio_write(sc, 0x14, 0x0fff);
        re_mdio_write(sc, 0x13, 0xa004);
        re_mdio_write(sc, 0x14, 0x0fff);
        re_mdio_write(sc, 0x13, 0xa002);
        re_mdio_write(sc, 0x14, 0x0fff);
        re_mdio_write(sc, 0x13, 0xa000);
        re_mdio_write(sc, 0x14, 0x107c);
        re_mdio_write(sc, 0x13, 0xb820);
        re_mdio_write(sc, 0x14, 0x0210);

        re_mdio_write(sc, 0x1F, 0x0A43);
        re_mdio_write(sc, 0x13, 0x0000);
        re_mdio_write(sc, 0x14, 0x0000);
        re_mdio_write(sc, 0x1f, 0x0B82);
        PhyRegValue = re_mdio_read(sc, 0x17);
        PhyRegValue &= ~(BIT_0);
        re_mdio_write(sc, 0x17, PhyRegValue);
        re_mdio_write(sc, 0x1f, 0x0A43);
        re_mdio_write(sc, 0x13, 0x8146);
        re_mdio_write(sc, 0x14, 0x0000);

        re_clear_phy_mcu_patch_request(sc);
        if (sc->RequiredSecLanDonglePatch) {
                re_mdio_write(sc, 0x1F, 0x0A43);
                PhyRegValue = re_mdio_read(sc, 0x11);
                PhyRegValue &= ~(BIT_6);
                re_mdio_write(sc, 0x11, PhyRegValue);
        }
}


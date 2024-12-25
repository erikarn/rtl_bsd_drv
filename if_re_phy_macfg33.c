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

#include "if_re_phy_macfg33.h"

void
re_hw_phy_config_macfg33(struct re_softc *sc, bool phy_power_saving)
{
	uint16_t Data;

	re_mdio_write(sc, 0x1F, 0x0001);
	re_mdio_write(sc, 0x06, 0x4064);
	re_mdio_write(sc, 0x07, 0x2863);
	re_mdio_write(sc, 0x08, 0x059C);
	re_mdio_write(sc, 0x09, 0x26B4);
	re_mdio_write(sc, 0x0A, 0x6A19);
	re_mdio_write(sc, 0x0B, 0xDCC8);
	re_mdio_write(sc, 0x10, 0xF06D);
	re_mdio_write(sc, 0x14, 0x7F68);
	re_mdio_write(sc, 0x18, 0x7FD9);
	re_mdio_write(sc, 0x1C, 0xF0FF);
	re_mdio_write(sc, 0x1D, 0x3D9C);
	re_mdio_write(sc, 0x1F, 0x0003);
	re_mdio_write(sc, 0x12, 0xF49F);
	re_mdio_write(sc, 0x13, 0x070B);
	re_mdio_write(sc, 0x1A, 0x05AD);
	re_mdio_write(sc, 0x14, 0x94C0);

	re_mdio_write(sc, 0x1F, 0x0002);
	re_mdio_write(sc, 0x06, 0x5561);
	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8332);
	re_mdio_write(sc, 0x06, 0x5561);

	if (re_efuse_read(sc, 0x01) == 0xb1) {
		re_mdio_write(sc, 0x1F, 0x0002);
		re_mdio_write(sc, 0x05, 0x669A);
		re_mdio_write(sc, 0x1F, 0x0005);
		re_mdio_write(sc, 0x05, 0x8330);
		re_mdio_write(sc, 0x06, 0x669A);

		re_mdio_write(sc, 0x1F, 0x0002);
		Data = re_mdio_read(sc, 0x0D);
		if ((Data & 0x00FF) != 0x006C) {
			Data &= 0xFF00;
			re_mdio_write(sc, 0x1F, 0x0002);
			re_mdio_write(sc, 0x0D, Data | 0x0065);
			re_mdio_write(sc, 0x0D, Data | 0x0066);
			re_mdio_write(sc, 0x0D, Data | 0x0067);
			re_mdio_write(sc, 0x0D, Data | 0x0068);
			re_mdio_write(sc, 0x0D, Data | 0x0069);
			re_mdio_write(sc, 0x0D, Data | 0x006A);
			re_mdio_write(sc, 0x0D, Data | 0x006B);
			re_mdio_write(sc, 0x0D, Data | 0x006C);
		}
	} else {
		re_mdio_write(sc, 0x1F, 0x0002);
		re_mdio_write(sc, 0x05, 0x2642);
		re_mdio_write(sc, 0x1F, 0x0005);
		re_mdio_write(sc, 0x05, 0x8330);
		re_mdio_write(sc, 0x06, 0x2642);
	}

	if (re_efuse_read(sc, 0x30) == 0x98) {
		re_mdio_write(sc, 0x1F, 0x0000);
		re_mdio_write(sc, 0x11, re_mdio_read(sc, 0x11) & ~0x02);
		re_mdio_write(sc, 0x1F, 0x0005);
		re_mdio_write(sc, 0x01, re_mdio_read(sc, 0x01) | 0x200);
	} else if (re_efuse_read(sc, 0x30) == 0x90) {
		re_mdio_write(sc, 0x1F, 0x0005);
		re_mdio_write(sc, 0x01, re_mdio_read(sc, 0x01) & ~0x200);
		re_mdio_write(sc, 0x1F, 0x0000);
		re_mdio_write(sc, 0x16, 0x5101);
	}

	re_mdio_write(sc, 0x1F, 0x0002);
	Data = re_mdio_read(sc, 0x02);
	Data &= ~0x600;
	Data |= 0x100;
	re_mdio_write(sc, 0x02, Data);
	Data = re_mdio_read(sc, 0x03);
	Data &= ~0xE000;
	re_mdio_write(sc, 0x03, Data);

	re_mdio_write(sc, 0x1F, 0x0001);
	re_mdio_write(sc, 0x17, 0x0CC0);

	re_mdio_write(sc, 0x1F, 0x0002);
	Data = re_mdio_read(sc, 0x0F);
	Data |= 0x17;
	re_mdio_write(sc, 0x0F, Data);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x001B);
	if (re_mdio_read(sc, 0x06) == 0xB300) {
		re_mdio_write(sc, 0x1f, 0x0005);
		re_mdio_write(sc, 0x05, 0xfff6);
		re_mdio_write(sc, 0x06, 0x0080);
		re_mdio_write(sc, 0x05, 0x8000);
		re_mdio_write(sc, 0x06, 0xf8f9);
		re_mdio_write(sc, 0x06, 0xfaee);
		re_mdio_write(sc, 0x06, 0xf8ea);
		re_mdio_write(sc, 0x06, 0x00ee);
		re_mdio_write(sc, 0x06, 0xf8eb);
		re_mdio_write(sc, 0x06, 0x00e2);
		re_mdio_write(sc, 0x06, 0xf87c);
		re_mdio_write(sc, 0x06, 0xe3f8);
		re_mdio_write(sc, 0x06, 0x7da5);
		re_mdio_write(sc, 0x06, 0x1111);
		re_mdio_write(sc, 0x06, 0x12d2);
		re_mdio_write(sc, 0x06, 0x40d6);
		re_mdio_write(sc, 0x06, 0x4444);
		re_mdio_write(sc, 0x06, 0x0281);
		re_mdio_write(sc, 0x06, 0xc6d2);
		re_mdio_write(sc, 0x06, 0xa0d6);
		re_mdio_write(sc, 0x06, 0xaaaa);
		re_mdio_write(sc, 0x06, 0x0281);
		re_mdio_write(sc, 0x06, 0xc6ae);
		re_mdio_write(sc, 0x06, 0x0fa5);
		re_mdio_write(sc, 0x06, 0x4444);
		re_mdio_write(sc, 0x06, 0x02ae);
		re_mdio_write(sc, 0x06, 0x4da5);
		re_mdio_write(sc, 0x06, 0xaaaa);
		re_mdio_write(sc, 0x06, 0x02ae);
		re_mdio_write(sc, 0x06, 0x47af);
		re_mdio_write(sc, 0x06, 0x81c2);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4e00);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4d0f);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4c0f);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4f00);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x5100);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4aff);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4bff);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x30e1);
		re_mdio_write(sc, 0x06, 0x8331);
		re_mdio_write(sc, 0x06, 0x58fe);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x8ae5);
		re_mdio_write(sc, 0x06, 0xf88b);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x32e1);
		re_mdio_write(sc, 0x06, 0x8333);
		re_mdio_write(sc, 0x06, 0x590f);
		re_mdio_write(sc, 0x06, 0xe283);
		re_mdio_write(sc, 0x06, 0x4d0c);
		re_mdio_write(sc, 0x06, 0x245a);
		re_mdio_write(sc, 0x06, 0xf01e);
		re_mdio_write(sc, 0x06, 0x12e4);
		re_mdio_write(sc, 0x06, 0xf88c);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x8daf);
		re_mdio_write(sc, 0x06, 0x81c2);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4f10);
		re_mdio_write(sc, 0x06, 0xe483);
		re_mdio_write(sc, 0x06, 0x4fe0);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x7800);
		re_mdio_write(sc, 0x06, 0x9f0a);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4fa0);
		re_mdio_write(sc, 0x06, 0x10a5);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4e01);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4e78);
		re_mdio_write(sc, 0x06, 0x059e);
		re_mdio_write(sc, 0x06, 0x9ae0);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x7804);
		re_mdio_write(sc, 0x06, 0x9e10);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4e78);
		re_mdio_write(sc, 0x06, 0x039e);
		re_mdio_write(sc, 0x06, 0x0fe0);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x7801);
		re_mdio_write(sc, 0x06, 0x9e05);
		re_mdio_write(sc, 0x06, 0xae0c);
		re_mdio_write(sc, 0x06, 0xaf81);
		re_mdio_write(sc, 0x06, 0xa7af);
		re_mdio_write(sc, 0x06, 0x8152);
		re_mdio_write(sc, 0x06, 0xaf81);
		re_mdio_write(sc, 0x06, 0x8baf);
		re_mdio_write(sc, 0x06, 0x81c2);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4800);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4900);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x5110);
		re_mdio_write(sc, 0x06, 0xe483);
		re_mdio_write(sc, 0x06, 0x5158);
		re_mdio_write(sc, 0x06, 0x019f);
		re_mdio_write(sc, 0x06, 0xead0);
		re_mdio_write(sc, 0x06, 0x00d1);
		re_mdio_write(sc, 0x06, 0x801f);
		re_mdio_write(sc, 0x06, 0x66e2);
		re_mdio_write(sc, 0x06, 0xf8ea);
		re_mdio_write(sc, 0x06, 0xe3f8);
		re_mdio_write(sc, 0x06, 0xeb5a);
		re_mdio_write(sc, 0x06, 0xf81e);
		re_mdio_write(sc, 0x06, 0x20e6);
		re_mdio_write(sc, 0x06, 0xf8ea);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0xebd3);
		re_mdio_write(sc, 0x06, 0x02b3);
		re_mdio_write(sc, 0x06, 0xfee2);
		re_mdio_write(sc, 0x06, 0xf87c);
		re_mdio_write(sc, 0x06, 0xef32);
		re_mdio_write(sc, 0x06, 0x5b80);
		re_mdio_write(sc, 0x06, 0xe3f8);
		re_mdio_write(sc, 0x06, 0x7d9e);
		re_mdio_write(sc, 0x06, 0x037d);
		re_mdio_write(sc, 0x06, 0xffff);
		re_mdio_write(sc, 0x06, 0x0d58);
		re_mdio_write(sc, 0x06, 0x1c55);
		re_mdio_write(sc, 0x06, 0x1a65);
		re_mdio_write(sc, 0x06, 0x11a1);
		re_mdio_write(sc, 0x06, 0x90d3);
		re_mdio_write(sc, 0x06, 0xe283);
		re_mdio_write(sc, 0x06, 0x48e3);
		re_mdio_write(sc, 0x06, 0x8349);
		re_mdio_write(sc, 0x06, 0x1b56);
		re_mdio_write(sc, 0x06, 0xab08);
		re_mdio_write(sc, 0x06, 0xef56);
		re_mdio_write(sc, 0x06, 0xe683);
		re_mdio_write(sc, 0x06, 0x48e7);
		re_mdio_write(sc, 0x06, 0x8349);
		re_mdio_write(sc, 0x06, 0x10d1);
		re_mdio_write(sc, 0x06, 0x801f);
		re_mdio_write(sc, 0x06, 0x66a0);
		re_mdio_write(sc, 0x06, 0x04b9);
		re_mdio_write(sc, 0x06, 0xe283);
		re_mdio_write(sc, 0x06, 0x48e3);
		re_mdio_write(sc, 0x06, 0x8349);
		re_mdio_write(sc, 0x06, 0xef65);
		re_mdio_write(sc, 0x06, 0xe283);
		re_mdio_write(sc, 0x06, 0x4ae3);
		re_mdio_write(sc, 0x06, 0x834b);
		re_mdio_write(sc, 0x06, 0x1b56);
		re_mdio_write(sc, 0x06, 0xaa0e);
		re_mdio_write(sc, 0x06, 0xef56);
		re_mdio_write(sc, 0x06, 0xe683);
		re_mdio_write(sc, 0x06, 0x4ae7);
		re_mdio_write(sc, 0x06, 0x834b);
		re_mdio_write(sc, 0x06, 0xe283);
		re_mdio_write(sc, 0x06, 0x4de6);
		re_mdio_write(sc, 0x06, 0x834c);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4da0);
		re_mdio_write(sc, 0x06, 0x000c);
		re_mdio_write(sc, 0x06, 0xaf81);
		re_mdio_write(sc, 0x06, 0x8be0);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0x10e4);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0xae04);
		re_mdio_write(sc, 0x06, 0x80e4);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4e78);
		re_mdio_write(sc, 0x06, 0x039e);
		re_mdio_write(sc, 0x06, 0x0be0);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x7804);
		re_mdio_write(sc, 0x06, 0x9e04);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4e02);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x32e1);
		re_mdio_write(sc, 0x06, 0x8333);
		re_mdio_write(sc, 0x06, 0x590f);
		re_mdio_write(sc, 0x06, 0xe283);
		re_mdio_write(sc, 0x06, 0x4d0c);
		re_mdio_write(sc, 0x06, 0x245a);
		re_mdio_write(sc, 0x06, 0xf01e);
		re_mdio_write(sc, 0x06, 0x12e4);
		re_mdio_write(sc, 0x06, 0xf88c);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x8de0);
		re_mdio_write(sc, 0x06, 0x8330);
		re_mdio_write(sc, 0x06, 0xe183);
		re_mdio_write(sc, 0x06, 0x3168);
		re_mdio_write(sc, 0x06, 0x01e4);
		re_mdio_write(sc, 0x06, 0xf88a);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x8bae);
		re_mdio_write(sc, 0x06, 0x37ee);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x03e0);
		re_mdio_write(sc, 0x06, 0x834c);
		re_mdio_write(sc, 0x06, 0xe183);
		re_mdio_write(sc, 0x06, 0x4d1b);
		re_mdio_write(sc, 0x06, 0x019e);
		re_mdio_write(sc, 0x06, 0x04aa);
		re_mdio_write(sc, 0x06, 0xa1ae);
		re_mdio_write(sc, 0x06, 0xa8ee);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x04ee);
		re_mdio_write(sc, 0x06, 0x834f);
		re_mdio_write(sc, 0x06, 0x00ae);
		re_mdio_write(sc, 0x06, 0xabe0);
		re_mdio_write(sc, 0x06, 0x834f);
		re_mdio_write(sc, 0x06, 0x7803);
		re_mdio_write(sc, 0x06, 0x9f14);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4e05);
		re_mdio_write(sc, 0x06, 0xd240);
		re_mdio_write(sc, 0x06, 0xd655);
		re_mdio_write(sc, 0x06, 0x5402);
		re_mdio_write(sc, 0x06, 0x81c6);
		re_mdio_write(sc, 0x06, 0xd2a0);
		re_mdio_write(sc, 0x06, 0xd6ba);
		re_mdio_write(sc, 0x06, 0x0002);
		re_mdio_write(sc, 0x06, 0x81c6);
		re_mdio_write(sc, 0x06, 0xfefd);
		re_mdio_write(sc, 0x06, 0xfc05);
		re_mdio_write(sc, 0x06, 0xf8e0);
		re_mdio_write(sc, 0x06, 0xf860);
		re_mdio_write(sc, 0x06, 0xe1f8);
		re_mdio_write(sc, 0x06, 0x6168);
		re_mdio_write(sc, 0x06, 0x02e4);
		re_mdio_write(sc, 0x06, 0xf860);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x61e0);
		re_mdio_write(sc, 0x06, 0xf848);
		re_mdio_write(sc, 0x06, 0xe1f8);
		re_mdio_write(sc, 0x06, 0x4958);
		re_mdio_write(sc, 0x06, 0x0f1e);
		re_mdio_write(sc, 0x06, 0x02e4);
		re_mdio_write(sc, 0x06, 0xf848);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x49d0);
		re_mdio_write(sc, 0x06, 0x0002);
		re_mdio_write(sc, 0x06, 0x820a);
		re_mdio_write(sc, 0x06, 0xbf83);
		re_mdio_write(sc, 0x06, 0x50ef);
		re_mdio_write(sc, 0x06, 0x46dc);
		re_mdio_write(sc, 0x06, 0x19dd);
		re_mdio_write(sc, 0x06, 0xd001);
		re_mdio_write(sc, 0x06, 0x0282);
		re_mdio_write(sc, 0x06, 0x0a02);
		re_mdio_write(sc, 0x06, 0x8226);
		re_mdio_write(sc, 0x06, 0xe0f8);
		re_mdio_write(sc, 0x06, 0x60e1);
		re_mdio_write(sc, 0x06, 0xf861);
		re_mdio_write(sc, 0x06, 0x58fd);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x60e5);
		re_mdio_write(sc, 0x06, 0xf861);
		re_mdio_write(sc, 0x06, 0xfc04);
		re_mdio_write(sc, 0x06, 0xf9fa);
		re_mdio_write(sc, 0x06, 0xfbc6);
		re_mdio_write(sc, 0x06, 0xbff8);
		re_mdio_write(sc, 0x06, 0x40be);
		re_mdio_write(sc, 0x06, 0x8350);
		re_mdio_write(sc, 0x06, 0xa001);
		re_mdio_write(sc, 0x06, 0x0107);
		re_mdio_write(sc, 0x06, 0x1b89);
		re_mdio_write(sc, 0x06, 0xcfd2);
		re_mdio_write(sc, 0x06, 0x08eb);
		re_mdio_write(sc, 0x06, 0xdb19);
		re_mdio_write(sc, 0x06, 0xb2fb);
		re_mdio_write(sc, 0x06, 0xfffe);
		re_mdio_write(sc, 0x06, 0xfd04);
		re_mdio_write(sc, 0x06, 0xf8e0);
		re_mdio_write(sc, 0x06, 0xf848);
		re_mdio_write(sc, 0x06, 0xe1f8);
		re_mdio_write(sc, 0x06, 0x4968);
		re_mdio_write(sc, 0x06, 0x08e4);
		re_mdio_write(sc, 0x06, 0xf848);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x4958);
		re_mdio_write(sc, 0x06, 0xf7e4);
		re_mdio_write(sc, 0x06, 0xf848);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x49fc);
		re_mdio_write(sc, 0x06, 0x044d);
		re_mdio_write(sc, 0x06, 0x2000);
		re_mdio_write(sc, 0x06, 0x024e);
		re_mdio_write(sc, 0x06, 0x2200);
		re_mdio_write(sc, 0x06, 0x024d);
		re_mdio_write(sc, 0x06, 0xdfff);
		re_mdio_write(sc, 0x06, 0x014e);
		re_mdio_write(sc, 0x06, 0xddff);
		re_mdio_write(sc, 0x06, 0x01f8);
		re_mdio_write(sc, 0x06, 0xfafb);
		re_mdio_write(sc, 0x06, 0xef79);
		re_mdio_write(sc, 0x06, 0xbff8);
		re_mdio_write(sc, 0x06, 0x22d8);
		re_mdio_write(sc, 0x06, 0x19d9);
		re_mdio_write(sc, 0x06, 0x5884);
		re_mdio_write(sc, 0x06, 0x9f09);
		re_mdio_write(sc, 0x06, 0xbf82);
		re_mdio_write(sc, 0x06, 0x6dd6);
		re_mdio_write(sc, 0x06, 0x8275);
		re_mdio_write(sc, 0x06, 0x0201);
		re_mdio_write(sc, 0x06, 0x4fef);
		re_mdio_write(sc, 0x06, 0x97ff);
		re_mdio_write(sc, 0x06, 0xfefc);
		re_mdio_write(sc, 0x06, 0x0517);
		re_mdio_write(sc, 0x06, 0xfffe);
		re_mdio_write(sc, 0x06, 0x0117);
		re_mdio_write(sc, 0x06, 0x0001);
		re_mdio_write(sc, 0x06, 0x0200);
		re_mdio_write(sc, 0x05, 0x83d8);
		re_mdio_write(sc, 0x06, 0x8000);
		re_mdio_write(sc, 0x05, 0x83d6);
		re_mdio_write(sc, 0x06, 0x824f);
		re_mdio_write(sc, 0x02, 0x2010);
		re_mdio_write(sc, 0x03, 0xdc00);
		re_mdio_write(sc, 0x1f, 0x0000);
		re_mdio_write(sc, 0x0b, 0x0600);
		re_mdio_write(sc, 0x1f, 0x0005);
		re_mdio_write(sc, 0x05, 0xfff6);
		re_mdio_write(sc, 0x06, 0x00fc);
		re_mdio_write(sc, 0x1f, 0x0000);
	}

	re_mdio_write(sc, 0x1F, 0x0000);
	re_mdio_write(sc, 0x0D, 0xF880);
	re_mdio_write(sc, 0x1F, 0x0000);
}

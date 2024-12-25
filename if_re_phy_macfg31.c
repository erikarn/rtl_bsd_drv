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

#include "if_re_phy_macfg31.h"

void
re_hw_phy_config_macfg31(struct re_softc *sc, bool phy_power_saving)
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
	Data = re_mdio_read(sc, 0x0B) & 0xFF00;
	Data |= 0x10;
	re_mdio_write(sc, 0x0B, Data);
	Data = re_mdio_read(sc, 0x0C) & 0x00FF;
	Data |= 0xA200;
	re_mdio_write(sc, 0x0C, Data);

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
		re_mdio_write(sc, 0x05, 0x6662);
		re_mdio_write(sc, 0x1F, 0x0005);
		re_mdio_write(sc, 0x05, 0x8330);
		re_mdio_write(sc, 0x06, 0x6662);
	}

	re_mdio_write(sc, 0x1F, 0x0002);
	Data = re_mdio_read(sc, 0x0D);
	Data |= 0x300;
	re_mdio_write(sc, 0x0D, Data);
	Data = re_mdio_read(sc, 0x0F);
	Data |= 0x10;
	re_mdio_write(sc, 0x0F, Data);

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

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x001B);
	if (re_mdio_read(sc, 0x06) == 0xBF00) {
		re_mdio_write(sc, 0x1f, 0x0005);
		re_mdio_write(sc, 0x05, 0xfff6);
		re_mdio_write(sc, 0x06, 0x0080);
		re_mdio_write(sc, 0x05, 0x8000);
		re_mdio_write(sc, 0x06, 0xf8f9);
		re_mdio_write(sc, 0x06, 0xfaef);
		re_mdio_write(sc, 0x06, 0x59ee);
		re_mdio_write(sc, 0x06, 0xf8ea);
		re_mdio_write(sc, 0x06, 0x00ee);
		re_mdio_write(sc, 0x06, 0xf8eb);
		re_mdio_write(sc, 0x06, 0x00e0);
		re_mdio_write(sc, 0x06, 0xf87c);
		re_mdio_write(sc, 0x06, 0xe1f8);
		re_mdio_write(sc, 0x06, 0x7d59);
		re_mdio_write(sc, 0x06, 0x0fef);
		re_mdio_write(sc, 0x06, 0x0139);
		re_mdio_write(sc, 0x06, 0x029e);
		re_mdio_write(sc, 0x06, 0x06ef);
		re_mdio_write(sc, 0x06, 0x1039);
		re_mdio_write(sc, 0x06, 0x089f);
		re_mdio_write(sc, 0x06, 0x2aee);
		re_mdio_write(sc, 0x06, 0xf8ea);
		re_mdio_write(sc, 0x06, 0x00ee);
		re_mdio_write(sc, 0x06, 0xf8eb);
		re_mdio_write(sc, 0x06, 0x01e0);
		re_mdio_write(sc, 0x06, 0xf87c);
		re_mdio_write(sc, 0x06, 0xe1f8);
		re_mdio_write(sc, 0x06, 0x7d58);
		re_mdio_write(sc, 0x06, 0x409e);
		re_mdio_write(sc, 0x06, 0x0f39);
		re_mdio_write(sc, 0x06, 0x46aa);
		re_mdio_write(sc, 0x06, 0x0bbf);
		re_mdio_write(sc, 0x06, 0x8290);
		re_mdio_write(sc, 0x06, 0xd682);
		re_mdio_write(sc, 0x06, 0x9802);
		re_mdio_write(sc, 0x06, 0x014f);
		re_mdio_write(sc, 0x06, 0xae09);
		re_mdio_write(sc, 0x06, 0xbf82);
		re_mdio_write(sc, 0x06, 0x98d6);
		re_mdio_write(sc, 0x06, 0x82a0);
		re_mdio_write(sc, 0x06, 0x0201);
		re_mdio_write(sc, 0x06, 0x4fef);
		re_mdio_write(sc, 0x06, 0x95fe);
		re_mdio_write(sc, 0x06, 0xfdfc);
		re_mdio_write(sc, 0x06, 0x05f8);
		re_mdio_write(sc, 0x06, 0xf9fa);
		re_mdio_write(sc, 0x06, 0xeef8);
		re_mdio_write(sc, 0x06, 0xea00);
		re_mdio_write(sc, 0x06, 0xeef8);
		re_mdio_write(sc, 0x06, 0xeb00);
		re_mdio_write(sc, 0x06, 0xe2f8);
		re_mdio_write(sc, 0x06, 0x7ce3);
		re_mdio_write(sc, 0x06, 0xf87d);
		re_mdio_write(sc, 0x06, 0xa511);
		re_mdio_write(sc, 0x06, 0x1112);
		re_mdio_write(sc, 0x06, 0xd240);
		re_mdio_write(sc, 0x06, 0xd644);
		re_mdio_write(sc, 0x06, 0x4402);
		re_mdio_write(sc, 0x06, 0x8217);
		re_mdio_write(sc, 0x06, 0xd2a0);
		re_mdio_write(sc, 0x06, 0xd6aa);
		re_mdio_write(sc, 0x06, 0xaa02);
		re_mdio_write(sc, 0x06, 0x8217);
		re_mdio_write(sc, 0x06, 0xae0f);
		re_mdio_write(sc, 0x06, 0xa544);
		re_mdio_write(sc, 0x06, 0x4402);
		re_mdio_write(sc, 0x06, 0xae4d);
		re_mdio_write(sc, 0x06, 0xa5aa);
		re_mdio_write(sc, 0x06, 0xaa02);
		re_mdio_write(sc, 0x06, 0xae47);
		re_mdio_write(sc, 0x06, 0xaf82);
		re_mdio_write(sc, 0x06, 0x13ee);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x00ee);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0x0fee);
		re_mdio_write(sc, 0x06, 0x834c);
		re_mdio_write(sc, 0x06, 0x0fee);
		re_mdio_write(sc, 0x06, 0x834f);
		re_mdio_write(sc, 0x06, 0x00ee);
		re_mdio_write(sc, 0x06, 0x8351);
		re_mdio_write(sc, 0x06, 0x00ee);
		re_mdio_write(sc, 0x06, 0x834a);
		re_mdio_write(sc, 0x06, 0xffee);
		re_mdio_write(sc, 0x06, 0x834b);
		re_mdio_write(sc, 0x06, 0xffe0);
		re_mdio_write(sc, 0x06, 0x8330);
		re_mdio_write(sc, 0x06, 0xe183);
		re_mdio_write(sc, 0x06, 0x3158);
		re_mdio_write(sc, 0x06, 0xfee4);
		re_mdio_write(sc, 0x06, 0xf88a);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x8be0);
		re_mdio_write(sc, 0x06, 0x8332);
		re_mdio_write(sc, 0x06, 0xe183);
		re_mdio_write(sc, 0x06, 0x3359);
		re_mdio_write(sc, 0x06, 0x0fe2);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0x0c24);
		re_mdio_write(sc, 0x06, 0x5af0);
		re_mdio_write(sc, 0x06, 0x1e12);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x8ce5);
		re_mdio_write(sc, 0x06, 0xf88d);
		re_mdio_write(sc, 0x06, 0xaf82);
		re_mdio_write(sc, 0x06, 0x13e0);
		re_mdio_write(sc, 0x06, 0x834f);
		re_mdio_write(sc, 0x06, 0x10e4);
		re_mdio_write(sc, 0x06, 0x834f);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4e78);
		re_mdio_write(sc, 0x06, 0x009f);
		re_mdio_write(sc, 0x06, 0x0ae0);
		re_mdio_write(sc, 0x06, 0x834f);
		re_mdio_write(sc, 0x06, 0xa010);
		re_mdio_write(sc, 0x06, 0xa5ee);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x01e0);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x7805);
		re_mdio_write(sc, 0x06, 0x9e9a);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4e78);
		re_mdio_write(sc, 0x06, 0x049e);
		re_mdio_write(sc, 0x06, 0x10e0);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x7803);
		re_mdio_write(sc, 0x06, 0x9e0f);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4e78);
		re_mdio_write(sc, 0x06, 0x019e);
		re_mdio_write(sc, 0x06, 0x05ae);
		re_mdio_write(sc, 0x06, 0x0caf);
		re_mdio_write(sc, 0x06, 0x81f8);
		re_mdio_write(sc, 0x06, 0xaf81);
		re_mdio_write(sc, 0x06, 0xa3af);
		re_mdio_write(sc, 0x06, 0x81dc);
		re_mdio_write(sc, 0x06, 0xaf82);
		re_mdio_write(sc, 0x06, 0x13ee);
		re_mdio_write(sc, 0x06, 0x8348);
		re_mdio_write(sc, 0x06, 0x00ee);
		re_mdio_write(sc, 0x06, 0x8349);
		re_mdio_write(sc, 0x06, 0x00e0);
		re_mdio_write(sc, 0x06, 0x8351);
		re_mdio_write(sc, 0x06, 0x10e4);
		re_mdio_write(sc, 0x06, 0x8351);
		re_mdio_write(sc, 0x06, 0x5801);
		re_mdio_write(sc, 0x06, 0x9fea);
		re_mdio_write(sc, 0x06, 0xd000);
		re_mdio_write(sc, 0x06, 0xd180);
		re_mdio_write(sc, 0x06, 0x1f66);
		re_mdio_write(sc, 0x06, 0xe2f8);
		re_mdio_write(sc, 0x06, 0xeae3);
		re_mdio_write(sc, 0x06, 0xf8eb);
		re_mdio_write(sc, 0x06, 0x5af8);
		re_mdio_write(sc, 0x06, 0x1e20);
		re_mdio_write(sc, 0x06, 0xe6f8);
		re_mdio_write(sc, 0x06, 0xeae5);
		re_mdio_write(sc, 0x06, 0xf8eb);
		re_mdio_write(sc, 0x06, 0xd302);
		re_mdio_write(sc, 0x06, 0xb3fe);
		re_mdio_write(sc, 0x06, 0xe2f8);
		re_mdio_write(sc, 0x06, 0x7cef);
		re_mdio_write(sc, 0x06, 0x325b);
		re_mdio_write(sc, 0x06, 0x80e3);
		re_mdio_write(sc, 0x06, 0xf87d);
		re_mdio_write(sc, 0x06, 0x9e03);
		re_mdio_write(sc, 0x06, 0x7dff);
		re_mdio_write(sc, 0x06, 0xff0d);
		re_mdio_write(sc, 0x06, 0x581c);
		re_mdio_write(sc, 0x06, 0x551a);
		re_mdio_write(sc, 0x06, 0x6511);
		re_mdio_write(sc, 0x06, 0xa190);
		re_mdio_write(sc, 0x06, 0xd3e2);
		re_mdio_write(sc, 0x06, 0x8348);
		re_mdio_write(sc, 0x06, 0xe383);
		re_mdio_write(sc, 0x06, 0x491b);
		re_mdio_write(sc, 0x06, 0x56ab);
		re_mdio_write(sc, 0x06, 0x08ef);
		re_mdio_write(sc, 0x06, 0x56e6);
		re_mdio_write(sc, 0x06, 0x8348);
		re_mdio_write(sc, 0x06, 0xe783);
		re_mdio_write(sc, 0x06, 0x4910);
		re_mdio_write(sc, 0x06, 0xd180);
		re_mdio_write(sc, 0x06, 0x1f66);
		re_mdio_write(sc, 0x06, 0xa004);
		re_mdio_write(sc, 0x06, 0xb9e2);
		re_mdio_write(sc, 0x06, 0x8348);
		re_mdio_write(sc, 0x06, 0xe383);
		re_mdio_write(sc, 0x06, 0x49ef);
		re_mdio_write(sc, 0x06, 0x65e2);
		re_mdio_write(sc, 0x06, 0x834a);
		re_mdio_write(sc, 0x06, 0xe383);
		re_mdio_write(sc, 0x06, 0x4b1b);
		re_mdio_write(sc, 0x06, 0x56aa);
		re_mdio_write(sc, 0x06, 0x0eef);
		re_mdio_write(sc, 0x06, 0x56e6);
		re_mdio_write(sc, 0x06, 0x834a);
		re_mdio_write(sc, 0x06, 0xe783);
		re_mdio_write(sc, 0x06, 0x4be2);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0xe683);
		re_mdio_write(sc, 0x06, 0x4ce0);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0xa000);
		re_mdio_write(sc, 0x06, 0x0caf);
		re_mdio_write(sc, 0x06, 0x81dc);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4d10);
		re_mdio_write(sc, 0x06, 0xe483);
		re_mdio_write(sc, 0x06, 0x4dae);
		re_mdio_write(sc, 0x06, 0x0480);
		re_mdio_write(sc, 0x06, 0xe483);
		re_mdio_write(sc, 0x06, 0x4de0);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x7803);
		re_mdio_write(sc, 0x06, 0x9e0b);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4e78);
		re_mdio_write(sc, 0x06, 0x049e);
		re_mdio_write(sc, 0x06, 0x04ee);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x02e0);
		re_mdio_write(sc, 0x06, 0x8332);
		re_mdio_write(sc, 0x06, 0xe183);
		re_mdio_write(sc, 0x06, 0x3359);
		re_mdio_write(sc, 0x06, 0x0fe2);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0x0c24);
		re_mdio_write(sc, 0x06, 0x5af0);
		re_mdio_write(sc, 0x06, 0x1e12);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x8ce5);
		re_mdio_write(sc, 0x06, 0xf88d);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x30e1);
		re_mdio_write(sc, 0x06, 0x8331);
		re_mdio_write(sc, 0x06, 0x6801);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x8ae5);
		re_mdio_write(sc, 0x06, 0xf88b);
		re_mdio_write(sc, 0x06, 0xae37);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4e03);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4ce1);
		re_mdio_write(sc, 0x06, 0x834d);
		re_mdio_write(sc, 0x06, 0x1b01);
		re_mdio_write(sc, 0x06, 0x9e04);
		re_mdio_write(sc, 0x06, 0xaaa1);
		re_mdio_write(sc, 0x06, 0xaea8);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4e04);
		re_mdio_write(sc, 0x06, 0xee83);
		re_mdio_write(sc, 0x06, 0x4f00);
		re_mdio_write(sc, 0x06, 0xaeab);
		re_mdio_write(sc, 0x06, 0xe083);
		re_mdio_write(sc, 0x06, 0x4f78);
		re_mdio_write(sc, 0x06, 0x039f);
		re_mdio_write(sc, 0x06, 0x14ee);
		re_mdio_write(sc, 0x06, 0x834e);
		re_mdio_write(sc, 0x06, 0x05d2);
		re_mdio_write(sc, 0x06, 0x40d6);
		re_mdio_write(sc, 0x06, 0x5554);
		re_mdio_write(sc, 0x06, 0x0282);
		re_mdio_write(sc, 0x06, 0x17d2);
		re_mdio_write(sc, 0x06, 0xa0d6);
		re_mdio_write(sc, 0x06, 0xba00);
		re_mdio_write(sc, 0x06, 0x0282);
		re_mdio_write(sc, 0x06, 0x17fe);
		re_mdio_write(sc, 0x06, 0xfdfc);
		re_mdio_write(sc, 0x06, 0x05f8);
		re_mdio_write(sc, 0x06, 0xe0f8);
		re_mdio_write(sc, 0x06, 0x60e1);
		re_mdio_write(sc, 0x06, 0xf861);
		re_mdio_write(sc, 0x06, 0x6802);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x60e5);
		re_mdio_write(sc, 0x06, 0xf861);
		re_mdio_write(sc, 0x06, 0xe0f8);
		re_mdio_write(sc, 0x06, 0x48e1);
		re_mdio_write(sc, 0x06, 0xf849);
		re_mdio_write(sc, 0x06, 0x580f);
		re_mdio_write(sc, 0x06, 0x1e02);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x48e5);
		re_mdio_write(sc, 0x06, 0xf849);
		re_mdio_write(sc, 0x06, 0xd000);
		re_mdio_write(sc, 0x06, 0x0282);
		re_mdio_write(sc, 0x06, 0x5bbf);
		re_mdio_write(sc, 0x06, 0x8350);
		re_mdio_write(sc, 0x06, 0xef46);
		re_mdio_write(sc, 0x06, 0xdc19);
		re_mdio_write(sc, 0x06, 0xddd0);
		re_mdio_write(sc, 0x06, 0x0102);
		re_mdio_write(sc, 0x06, 0x825b);
		re_mdio_write(sc, 0x06, 0x0282);
		re_mdio_write(sc, 0x06, 0x77e0);
		re_mdio_write(sc, 0x06, 0xf860);
		re_mdio_write(sc, 0x06, 0xe1f8);
		re_mdio_write(sc, 0x06, 0x6158);
		re_mdio_write(sc, 0x06, 0xfde4);
		re_mdio_write(sc, 0x06, 0xf860);
		re_mdio_write(sc, 0x06, 0xe5f8);
		re_mdio_write(sc, 0x06, 0x61fc);
		re_mdio_write(sc, 0x06, 0x04f9);
		re_mdio_write(sc, 0x06, 0xfafb);
		re_mdio_write(sc, 0x06, 0xc6bf);
		re_mdio_write(sc, 0x06, 0xf840);
		re_mdio_write(sc, 0x06, 0xbe83);
		re_mdio_write(sc, 0x06, 0x50a0);
		re_mdio_write(sc, 0x06, 0x0101);
		re_mdio_write(sc, 0x06, 0x071b);
		re_mdio_write(sc, 0x06, 0x89cf);
		re_mdio_write(sc, 0x06, 0xd208);
		re_mdio_write(sc, 0x06, 0xebdb);
		re_mdio_write(sc, 0x06, 0x19b2);
		re_mdio_write(sc, 0x06, 0xfbff);
		re_mdio_write(sc, 0x06, 0xfefd);
		re_mdio_write(sc, 0x06, 0x04f8);
		re_mdio_write(sc, 0x06, 0xe0f8);
		re_mdio_write(sc, 0x06, 0x48e1);
		re_mdio_write(sc, 0x06, 0xf849);
		re_mdio_write(sc, 0x06, 0x6808);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x48e5);
		re_mdio_write(sc, 0x06, 0xf849);
		re_mdio_write(sc, 0x06, 0x58f7);
		re_mdio_write(sc, 0x06, 0xe4f8);
		re_mdio_write(sc, 0x06, 0x48e5);
		re_mdio_write(sc, 0x06, 0xf849);
		re_mdio_write(sc, 0x06, 0xfc04);
		re_mdio_write(sc, 0x06, 0x4d20);
		re_mdio_write(sc, 0x06, 0x0002);
		re_mdio_write(sc, 0x06, 0x4e22);
		re_mdio_write(sc, 0x06, 0x0002);
		re_mdio_write(sc, 0x06, 0x4ddf);
		re_mdio_write(sc, 0x06, 0xff01);
		re_mdio_write(sc, 0x06, 0x4edd);
		re_mdio_write(sc, 0x06, 0xff01);
		re_mdio_write(sc, 0x06, 0xf8fa);
		re_mdio_write(sc, 0x06, 0xfbef);
		re_mdio_write(sc, 0x06, 0x79bf);
		re_mdio_write(sc, 0x06, 0xf822);
		re_mdio_write(sc, 0x06, 0xd819);
		re_mdio_write(sc, 0x06, 0xd958);
		re_mdio_write(sc, 0x06, 0x849f);
		re_mdio_write(sc, 0x06, 0x09bf);
		re_mdio_write(sc, 0x06, 0x82be);
		re_mdio_write(sc, 0x06, 0xd682);
		re_mdio_write(sc, 0x06, 0xc602);
		re_mdio_write(sc, 0x06, 0x014f);
		re_mdio_write(sc, 0x06, 0xef97);
		re_mdio_write(sc, 0x06, 0xfffe);
		re_mdio_write(sc, 0x06, 0xfc05);
		re_mdio_write(sc, 0x06, 0x17ff);
		re_mdio_write(sc, 0x06, 0xfe01);
		re_mdio_write(sc, 0x06, 0x1700);
		re_mdio_write(sc, 0x06, 0x0102);
		re_mdio_write(sc, 0x05, 0x83d8);
		re_mdio_write(sc, 0x06, 0x8051);
		re_mdio_write(sc, 0x05, 0x83d6);
		re_mdio_write(sc, 0x06, 0x82a0);
		re_mdio_write(sc, 0x05, 0x83d4);
		re_mdio_write(sc, 0x06, 0x8000);
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

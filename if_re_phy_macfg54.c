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

#include "if_re_phy_macfg54.h"

void
re_hw_phy_config_macfg54(struct re_softc *sc, bool phy_power_saving)
{
	uint32_t Data_u32;

	Data_u32 = re_eri_read(sc, 0x1D0, 4, ERIAR_ExGMAC);
	Data_u32 &= 0xFFFF0000;
	re_eri_write(sc, 0x1D0, 4, Data_u32, ERIAR_ExGMAC);

	if (sc->re_type == MACFG_55) {
		re_mdio_write(sc, 0x1F, 0x0000);
		re_mdio_write(sc, 0x18, 0x0310);
		re_mdio_write(sc, 0x1F, 0x0000);

		re_mdio_write(sc, 0x1f, 0x0004);
		re_mdio_write(sc, 0x1f, 0x0004);
		re_mdio_write(sc, 0x19, 0x7070);
		re_mdio_write(sc, 0x1c, 0x0600);
		re_mdio_write(sc, 0x1d, 0x9700);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x4007);
		re_mdio_write(sc, 0x1d, 0x4400);
		re_mdio_write(sc, 0x1d, 0x4800);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x5310);
		re_mdio_write(sc, 0x1d, 0x6000);
		re_mdio_write(sc, 0x1d, 0x6800);
		re_mdio_write(sc, 0x1d, 0x673e);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x571f);
		re_mdio_write(sc, 0x1d, 0x5ffb);
		re_mdio_write(sc, 0x1d, 0xaa04);
		re_mdio_write(sc, 0x1d, 0x5b58);
		re_mdio_write(sc, 0x1d, 0x6100);
		re_mdio_write(sc, 0x1d, 0x3016);
		re_mdio_write(sc, 0x1d, 0x5b64);
		re_mdio_write(sc, 0x1d, 0x6080);
		re_mdio_write(sc, 0x1d, 0xa6fa);
		re_mdio_write(sc, 0x1d, 0xdcdb);
		re_mdio_write(sc, 0x1d, 0x0015);
		re_mdio_write(sc, 0x1d, 0xb915);
		re_mdio_write(sc, 0x1d, 0xb511);
		re_mdio_write(sc, 0x1d, 0xd16b);
		re_mdio_write(sc, 0x1d, 0x000f);
		re_mdio_write(sc, 0x1d, 0xb40f);
		re_mdio_write(sc, 0x1d, 0xd06b);
		re_mdio_write(sc, 0x1d, 0x000d);
		re_mdio_write(sc, 0x1d, 0xb206);
		re_mdio_write(sc, 0x1d, 0x7c01);
		re_mdio_write(sc, 0x1d, 0x5800);
		re_mdio_write(sc, 0x1d, 0x7c04);
		re_mdio_write(sc, 0x1d, 0x5c00);
		re_mdio_write(sc, 0x1d, 0x3010);
		re_mdio_write(sc, 0x1d, 0x7c01);
		re_mdio_write(sc, 0x1d, 0x5801);
		re_mdio_write(sc, 0x1d, 0x7c04);
		re_mdio_write(sc, 0x1d, 0x5c04);
		re_mdio_write(sc, 0x1d, 0x3016);
		re_mdio_write(sc, 0x1d, 0x307e);
		re_mdio_write(sc, 0x1d, 0x30f4);
		re_mdio_write(sc, 0x1d, 0x319f);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4c60);
		re_mdio_write(sc, 0x1d, 0x6803);
		re_mdio_write(sc, 0x1d, 0x7d00);
		re_mdio_write(sc, 0x1d, 0x6900);
		re_mdio_write(sc, 0x1d, 0x6520);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0xaf03);
		re_mdio_write(sc, 0x1d, 0x6115);
		re_mdio_write(sc, 0x1d, 0x303a);
		re_mdio_write(sc, 0x1d, 0x6097);
		re_mdio_write(sc, 0x1d, 0x57e0);
		re_mdio_write(sc, 0x1d, 0x580c);
		re_mdio_write(sc, 0x1d, 0x588c);
		re_mdio_write(sc, 0x1d, 0x5f80);
		re_mdio_write(sc, 0x1d, 0x4827);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c10);
		re_mdio_write(sc, 0x1d, 0x8400);
		re_mdio_write(sc, 0x1d, 0x7c30);
		re_mdio_write(sc, 0x1d, 0x6020);
		re_mdio_write(sc, 0x1d, 0x48bf);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0xb802);
		re_mdio_write(sc, 0x1d, 0x3053);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x6808);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x7c10);
		re_mdio_write(sc, 0x1d, 0x6810);
		re_mdio_write(sc, 0x1d, 0xd6cf);
		re_mdio_write(sc, 0x1d, 0x0002);
		re_mdio_write(sc, 0x1d, 0x80fe);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4c80);
		re_mdio_write(sc, 0x1d, 0x7c10);
		re_mdio_write(sc, 0x1d, 0x6800);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x6800);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x7c23);
		re_mdio_write(sc, 0x1d, 0x5c23);
		re_mdio_write(sc, 0x1d, 0x481e);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c02);
		re_mdio_write(sc, 0x1d, 0x5310);
		re_mdio_write(sc, 0x1d, 0x81ff);
		re_mdio_write(sc, 0x1d, 0x30c1);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4d00);
		re_mdio_write(sc, 0x1d, 0x4832);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c10);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x6000);
		re_mdio_write(sc, 0x1d, 0xa4bd);
		re_mdio_write(sc, 0x1d, 0xd9b3);
		re_mdio_write(sc, 0x1d, 0x00fe);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4d20);
		re_mdio_write(sc, 0x1d, 0x7e00);
		re_mdio_write(sc, 0x1d, 0x6200);
		re_mdio_write(sc, 0x1d, 0x3001);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4dc0);
		re_mdio_write(sc, 0x1d, 0xd09d);
		re_mdio_write(sc, 0x1d, 0x0002);
		re_mdio_write(sc, 0x1d, 0xb4fe);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4d80);
		re_mdio_write(sc, 0x1d, 0x7c04);
		re_mdio_write(sc, 0x1d, 0x6004);
		re_mdio_write(sc, 0x1d, 0x6802);
		re_mdio_write(sc, 0x1d, 0x6728);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x6000);
		re_mdio_write(sc, 0x1d, 0x486c);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x9503);
		re_mdio_write(sc, 0x1d, 0x7e00);
		re_mdio_write(sc, 0x1d, 0x6200);
		re_mdio_write(sc, 0x1d, 0x571f);
		re_mdio_write(sc, 0x1d, 0x5fbb);
		re_mdio_write(sc, 0x1d, 0xaa05);
		re_mdio_write(sc, 0x1d, 0x5b58);
		re_mdio_write(sc, 0x1d, 0x7d80);
		re_mdio_write(sc, 0x1d, 0x6100);
		re_mdio_write(sc, 0x1d, 0x309a);
		re_mdio_write(sc, 0x1d, 0x5b64);
		re_mdio_write(sc, 0x1d, 0x7d80);
		re_mdio_write(sc, 0x1d, 0x6080);
		re_mdio_write(sc, 0x1d, 0xcdab);
		re_mdio_write(sc, 0x1d, 0x0058);
		re_mdio_write(sc, 0x1d, 0xcd8d);
		re_mdio_write(sc, 0x1d, 0x0056);
		re_mdio_write(sc, 0x1d, 0xd96b);
		re_mdio_write(sc, 0x1d, 0x0054);
		re_mdio_write(sc, 0x1d, 0xd0a0);
		re_mdio_write(sc, 0x1d, 0x00d8);
		re_mdio_write(sc, 0x1d, 0xcba0);
		re_mdio_write(sc, 0x1d, 0x0003);
		re_mdio_write(sc, 0x1d, 0x80ec);
		re_mdio_write(sc, 0x1d, 0x30a7);
		re_mdio_write(sc, 0x1d, 0x30b4);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4ce0);
		re_mdio_write(sc, 0x1d, 0x4832);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c08);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x6008);
		re_mdio_write(sc, 0x1d, 0x8300);
		re_mdio_write(sc, 0x1d, 0xb902);
		re_mdio_write(sc, 0x1d, 0x307e);
		re_mdio_write(sc, 0x1d, 0x3068);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4da0);
		re_mdio_write(sc, 0x1d, 0x6608);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x56a0);
		re_mdio_write(sc, 0x1d, 0x590c);
		re_mdio_write(sc, 0x1d, 0x5fa0);
		re_mdio_write(sc, 0x1d, 0xcba4);
		re_mdio_write(sc, 0x1d, 0x0004);
		re_mdio_write(sc, 0x1d, 0xcd8d);
		re_mdio_write(sc, 0x1d, 0x0002);
		re_mdio_write(sc, 0x1d, 0x80fc);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4ca0);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x6408);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x7d00);
		re_mdio_write(sc, 0x1d, 0x6800);
		re_mdio_write(sc, 0x1d, 0xb603);
		re_mdio_write(sc, 0x1d, 0x7c10);
		re_mdio_write(sc, 0x1d, 0x6010);
		re_mdio_write(sc, 0x1d, 0x7d1f);
		re_mdio_write(sc, 0x1d, 0x551f);
		re_mdio_write(sc, 0x1d, 0x5fb3);
		re_mdio_write(sc, 0x1d, 0xaa05);
		re_mdio_write(sc, 0x1d, 0x7c80);
		re_mdio_write(sc, 0x1d, 0x5800);
		re_mdio_write(sc, 0x1d, 0x5b58);
		re_mdio_write(sc, 0x1d, 0x30d7);
		re_mdio_write(sc, 0x1d, 0x7c80);
		re_mdio_write(sc, 0x1d, 0x5800);
		re_mdio_write(sc, 0x1d, 0x5b64);
		re_mdio_write(sc, 0x1d, 0x4827);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c10);
		re_mdio_write(sc, 0x1d, 0x8400);
		re_mdio_write(sc, 0x1d, 0x7c10);
		re_mdio_write(sc, 0x1d, 0x6000);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4cc0);
		re_mdio_write(sc, 0x1d, 0x7d00);
		re_mdio_write(sc, 0x1d, 0x6400);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x5fbb);
		re_mdio_write(sc, 0x1d, 0x4824);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c00);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c04);
		re_mdio_write(sc, 0x1d, 0x8200);
		re_mdio_write(sc, 0x1d, 0x7ce0);
		re_mdio_write(sc, 0x1d, 0x5400);
		re_mdio_write(sc, 0x1d, 0x7d00);
		re_mdio_write(sc, 0x1d, 0x6500);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x30a7);
		re_mdio_write(sc, 0x1d, 0x3001);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4e00);
		re_mdio_write(sc, 0x1d, 0x4007);
		re_mdio_write(sc, 0x1d, 0x4400);
		re_mdio_write(sc, 0x1d, 0x5310);
		re_mdio_write(sc, 0x1d, 0x6800);
		re_mdio_write(sc, 0x1d, 0x673e);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x570f);
		re_mdio_write(sc, 0x1d, 0x5fff);
		re_mdio_write(sc, 0x1d, 0xaa05);
		re_mdio_write(sc, 0x1d, 0x585b);
		re_mdio_write(sc, 0x1d, 0x7d80);
		re_mdio_write(sc, 0x1d, 0x6100);
		re_mdio_write(sc, 0x1d, 0x3107);
		re_mdio_write(sc, 0x1d, 0x5867);
		re_mdio_write(sc, 0x1d, 0x7d80);
		re_mdio_write(sc, 0x1d, 0x6080);
		re_mdio_write(sc, 0x1d, 0x9403);
		re_mdio_write(sc, 0x1d, 0x7e00);
		re_mdio_write(sc, 0x1d, 0x6200);
		re_mdio_write(sc, 0x1d, 0xcda3);
		re_mdio_write(sc, 0x1d, 0x00e8);
		re_mdio_write(sc, 0x1d, 0xcd85);
		re_mdio_write(sc, 0x1d, 0x00e6);
		re_mdio_write(sc, 0x1d, 0xd96b);
		re_mdio_write(sc, 0x1d, 0x00e4);
		re_mdio_write(sc, 0x1d, 0x96e4);
		re_mdio_write(sc, 0x1d, 0x6800);
		re_mdio_write(sc, 0x1d, 0x673e);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4e20);
		re_mdio_write(sc, 0x1d, 0x96dd);
		re_mdio_write(sc, 0x1d, 0x8b04);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x5008);
		re_mdio_write(sc, 0x1d, 0xab03);
		re_mdio_write(sc, 0x1d, 0x7c08);
		re_mdio_write(sc, 0x1d, 0x5000);
		re_mdio_write(sc, 0x1d, 0x6801);
		re_mdio_write(sc, 0x1d, 0x677e);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0xdb7c);
		re_mdio_write(sc, 0x1d, 0x00ee);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x7fe1);
		re_mdio_write(sc, 0x1d, 0x4e40);
		re_mdio_write(sc, 0x1d, 0x4837);
		re_mdio_write(sc, 0x1d, 0x4418);
		re_mdio_write(sc, 0x1d, 0x41c7);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4e40);
		re_mdio_write(sc, 0x1d, 0x7c40);
		re_mdio_write(sc, 0x1d, 0x5400);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x8fc2);
		re_mdio_write(sc, 0x1d, 0xd2a0);
		re_mdio_write(sc, 0x1d, 0x004b);
		re_mdio_write(sc, 0x1d, 0x9204);
		re_mdio_write(sc, 0x1d, 0xa042);
		re_mdio_write(sc, 0x1d, 0x3132);
		re_mdio_write(sc, 0x1d, 0x30f4);
		re_mdio_write(sc, 0x1d, 0x7fe1);
		re_mdio_write(sc, 0x1d, 0x4e60);
		re_mdio_write(sc, 0x1d, 0x489c);
		re_mdio_write(sc, 0x1d, 0x4628);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4e60);
		re_mdio_write(sc, 0x1d, 0x7e28);
		re_mdio_write(sc, 0x1d, 0x4628);
		re_mdio_write(sc, 0x1d, 0x7c40);
		re_mdio_write(sc, 0x1d, 0x5400);
		re_mdio_write(sc, 0x1d, 0x7c01);
		re_mdio_write(sc, 0x1d, 0x5800);
		re_mdio_write(sc, 0x1d, 0x7c04);
		re_mdio_write(sc, 0x1d, 0x5c00);
		re_mdio_write(sc, 0x1d, 0x41e8);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x8fa8);
		re_mdio_write(sc, 0x1d, 0xb241);
		re_mdio_write(sc, 0x1d, 0xa02a);
		re_mdio_write(sc, 0x1d, 0x314c);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4ea0);
		re_mdio_write(sc, 0x1d, 0x7c02);
		re_mdio_write(sc, 0x1d, 0x4402);
		re_mdio_write(sc, 0x1d, 0x4448);
		re_mdio_write(sc, 0x1d, 0x4894);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c03);
		re_mdio_write(sc, 0x1d, 0x4824);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c07);
		re_mdio_write(sc, 0x1d, 0x41ef);
		re_mdio_write(sc, 0x1d, 0x41ff);
		re_mdio_write(sc, 0x1d, 0x4891);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c07);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c17);
		re_mdio_write(sc, 0x1d, 0x8400);
		re_mdio_write(sc, 0x1d, 0x8ef8);
		re_mdio_write(sc, 0x1d, 0x41c7);
		re_mdio_write(sc, 0x1d, 0x8f8d);
		re_mdio_write(sc, 0x1d, 0x92d5);
		re_mdio_write(sc, 0x1d, 0xa10f);
		re_mdio_write(sc, 0x1d, 0xd480);
		re_mdio_write(sc, 0x1d, 0x0008);
		re_mdio_write(sc, 0x1d, 0xd580);
		re_mdio_write(sc, 0x1d, 0x00b8);
		re_mdio_write(sc, 0x1d, 0xa202);
		re_mdio_write(sc, 0x1d, 0x3167);
		re_mdio_write(sc, 0x1d, 0x7c04);
		re_mdio_write(sc, 0x1d, 0x4404);
		re_mdio_write(sc, 0x1d, 0x3167);
		re_mdio_write(sc, 0x1d, 0xd484);
		re_mdio_write(sc, 0x1d, 0x00f3);
		re_mdio_write(sc, 0x1d, 0xd484);
		re_mdio_write(sc, 0x1d, 0x00f1);
		re_mdio_write(sc, 0x1d, 0x30f4);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4ee0);
		re_mdio_write(sc, 0x1d, 0x7c40);
		re_mdio_write(sc, 0x1d, 0x5400);
		re_mdio_write(sc, 0x1d, 0x4488);
		re_mdio_write(sc, 0x1d, 0x41cf);
		re_mdio_write(sc, 0x1d, 0x30f4);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4ec0);
		re_mdio_write(sc, 0x1d, 0x48f3);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c09);
		re_mdio_write(sc, 0x1d, 0x4508);
		re_mdio_write(sc, 0x1d, 0x41c7);
		re_mdio_write(sc, 0x1d, 0x8fb0);
		re_mdio_write(sc, 0x1d, 0xd218);
		re_mdio_write(sc, 0x1d, 0x00ae);
		re_mdio_write(sc, 0x1d, 0xd2a4);
		re_mdio_write(sc, 0x1d, 0x009e);
		re_mdio_write(sc, 0x1d, 0x3188);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4e80);
		re_mdio_write(sc, 0x1d, 0x4832);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c01);
		re_mdio_write(sc, 0x1d, 0x7c1f);
		re_mdio_write(sc, 0x1d, 0x4c11);
		re_mdio_write(sc, 0x1d, 0x4428);
		re_mdio_write(sc, 0x1d, 0x7c40);
		re_mdio_write(sc, 0x1d, 0x5440);
		re_mdio_write(sc, 0x1d, 0x7c01);
		re_mdio_write(sc, 0x1d, 0x5801);
		re_mdio_write(sc, 0x1d, 0x7c04);
		re_mdio_write(sc, 0x1d, 0x5c04);
		re_mdio_write(sc, 0x1d, 0x41e8);
		re_mdio_write(sc, 0x1d, 0xa4b3);
		re_mdio_write(sc, 0x1d, 0x319d);
		re_mdio_write(sc, 0x1d, 0x7fe0);
		re_mdio_write(sc, 0x1d, 0x4f20);
		re_mdio_write(sc, 0x1d, 0x6800);
		re_mdio_write(sc, 0x1d, 0x673e);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x0000);
		re_mdio_write(sc, 0x1d, 0x570f);
		re_mdio_write(sc, 0x1d, 0x5fff);
		re_mdio_write(sc, 0x1d, 0xaa04);
		re_mdio_write(sc, 0x1d, 0x585b);
		re_mdio_write(sc, 0x1d, 0x6100);
		re_mdio_write(sc, 0x1d, 0x31ad);
		re_mdio_write(sc, 0x1d, 0x5867);
		re_mdio_write(sc, 0x1d, 0x6080);
		re_mdio_write(sc, 0x1d, 0xbcf2);
		re_mdio_write(sc, 0x1d, 0x3001);
		re_mdio_write(sc, 0x1f, 0x0004);
		re_mdio_write(sc, 0x1c, 0x0200);
		re_mdio_write(sc, 0x19, 0x7030);
		re_mdio_write(sc, 0x1f, 0x0000);
	}

	re_mdio_write(sc, 0x1F, 0x0001);
	re_mdio_write(sc, 0x11, 0x83BA);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_clear_eth_phy_bit(sc, 0x1A, BIT_2);
	re_mdio_write(sc, 0x1F, 0x0000);

	if (phy_power_saving == 1) {
		re_mdio_write(sc, 0x1F, 0x0000);
		re_mdio_write(sc, 0x18, 0x8310);
		re_mdio_write(sc, 0x1F, 0x0000);
	} else {
		re_mdio_write(sc, 0x1F, 0x0000);
		re_mdio_write(sc, 0x18, 0x0310);
		re_mdio_write(sc, 0x1F, 0x0000);
		DELAY(20000);
	}

	re_mdio_write(sc, 0x1F, 0x0000);
	re_mdio_write(sc, 0x0D, 0x0007);
	re_mdio_write(sc, 0x0E, 0x003C);
	re_mdio_write(sc, 0x0D, 0x4007);
	re_mdio_write(sc, 0x0E, 0x0000);
	re_mdio_write(sc, 0x0D, 0x0000);
	re_mdio_write(sc, 0x1F, 0x0000);
	re_mdio_write(sc, 0x0D, 0x0003);
	re_mdio_write(sc, 0x0E, 0x0015);
	re_mdio_write(sc, 0x0D, 0x4003);
	re_mdio_write(sc, 0x0E, 0x0000);
	re_mdio_write(sc, 0x0D, 0x0000);
}

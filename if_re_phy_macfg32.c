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

#include "if_re_phy_macfg32.h"

void
re_hw_phy_config_macfg32(struct re_softc *sc, bool phy_power_saving)
{

	re_mdio_write(sc, 0x1F, 0x0001);
	re_mdio_write(sc, 0x06, 0x4064);
	re_mdio_write(sc, 0x07, 0x2863);
	re_mdio_write(sc, 0x08, 0x059C);
	re_mdio_write(sc, 0x09, 0x26B4);
	re_mdio_write(sc, 0x0A, 0x6A19);
	re_mdio_write(sc, 0x0B, 0xBCC0);
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
	re_mdio_write(sc, 0x06, 0x5571);

	re_mdio_write(sc, 0x1F, 0x0002);
	re_mdio_write(sc, 0x05, 0x2642);

	re_mdio_write(sc, 0x1F, 0x0002);
	re_mdio_write(sc, 0x02, 0xC107);
	re_mdio_write(sc, 0x03, 0x1002);

	re_mdio_write(sc, 0x1F, 0x0001);
	re_mdio_write(sc, 0x16, 0x0CC0);

	re_mdio_write(sc, 0x1F, 0x0002);
	re_mdio_write(sc, 0x0F, 0x0017);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8200);
	re_mdio_write(sc, 0x06, 0xF8F9);
	re_mdio_write(sc, 0x06, 0xFAEF);
	re_mdio_write(sc, 0x06, 0x59EE);
	re_mdio_write(sc, 0x06, 0xF8EA);
	re_mdio_write(sc, 0x06, 0x00EE);
	re_mdio_write(sc, 0x06, 0xF8EB);
	re_mdio_write(sc, 0x06, 0x00E0);
	re_mdio_write(sc, 0x06, 0xF87C);
	re_mdio_write(sc, 0x06, 0xE1F8);
	re_mdio_write(sc, 0x06, 0x7D59);
	re_mdio_write(sc, 0x06, 0x0FEF);
	re_mdio_write(sc, 0x06, 0x0139);
	re_mdio_write(sc, 0x06, 0x029E);
	re_mdio_write(sc, 0x06, 0x06EF);
	re_mdio_write(sc, 0x06, 0x1039);
	re_mdio_write(sc, 0x06, 0x089F);
	re_mdio_write(sc, 0x06, 0x2AEE);
	re_mdio_write(sc, 0x06, 0xF8EA);
	re_mdio_write(sc, 0x06, 0x00EE);
	re_mdio_write(sc, 0x06, 0xF8EB);
	re_mdio_write(sc, 0x06, 0x01E0);
	re_mdio_write(sc, 0x06, 0xF87C);
	re_mdio_write(sc, 0x06, 0xE1F8);
	re_mdio_write(sc, 0x06, 0x7D58);
	re_mdio_write(sc, 0x06, 0x409E);
	re_mdio_write(sc, 0x06, 0x0F39);
	re_mdio_write(sc, 0x06, 0x46AA);
	re_mdio_write(sc, 0x06, 0x0BBF);
	re_mdio_write(sc, 0x06, 0x8251);
	re_mdio_write(sc, 0x06, 0xD682);
	re_mdio_write(sc, 0x06, 0x5902);
	re_mdio_write(sc, 0x06, 0x014F);
	re_mdio_write(sc, 0x06, 0xAE09);
	re_mdio_write(sc, 0x06, 0xBF82);
	re_mdio_write(sc, 0x06, 0x59D6);
	re_mdio_write(sc, 0x06, 0x8261);
	re_mdio_write(sc, 0x06, 0x0201);
	re_mdio_write(sc, 0x06, 0x4FEF);
	re_mdio_write(sc, 0x06, 0x95FE);
	re_mdio_write(sc, 0x06, 0xFDFC);
	re_mdio_write(sc, 0x06, 0x054D);
	re_mdio_write(sc, 0x06, 0x2000);
	re_mdio_write(sc, 0x06, 0x024E);
	re_mdio_write(sc, 0x06, 0x2200);
	re_mdio_write(sc, 0x06, 0x024D);
	re_mdio_write(sc, 0x06, 0xDFFF);
	re_mdio_write(sc, 0x06, 0x014E);
	re_mdio_write(sc, 0x06, 0xDDFF);
	re_mdio_write(sc, 0x06, 0x0100);
	re_mdio_write(sc, 0x02, 0x6010);
	re_mdio_write(sc, 0x05, 0xFFF6);
	re_mdio_write(sc, 0x06, 0x00EC);
	re_mdio_write(sc, 0x05, 0x83D4);
	re_mdio_write(sc, 0x06, 0x8200);

}

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

#include "if_re_phy_macfg59.h"

void
re_hw_phy_config_macfg59(struct re_softc *sc, bool phy_power_saving)
{

	re_mdio_write(sc, 0x1F, 0x0BCC);
	re_clear_eth_phy_bit(sc, 0x14, BIT_8);
	re_mdio_write(sc, 0x1F, 0x0A44);
	re_set_eth_phy_bit(sc, 0x11, BIT_7);
	re_set_eth_phy_bit(sc, 0x11, BIT_6);
	re_mdio_write(sc, 0x1F, 0x0A43);
	re_mdio_write(sc, 0x13, 0x8084);
	re_clear_eth_phy_bit(sc, 0x14, (BIT_14 | BIT_13));
	re_set_eth_phy_bit(sc, 0x10, BIT_12);
	re_set_eth_phy_bit(sc, 0x10, BIT_1);
	re_set_eth_phy_bit(sc, 0x10, BIT_0);

	re_mdio_write(sc, 0x1F, 0x0A43);
	re_mdio_write(sc, 0x13, 0x8012);
	re_set_eth_phy_bit(sc, 0x14, BIT_15);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0BCE);
	re_mdio_write(sc, 0x12, 0x8860);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0A43);
	re_mdio_write(sc, 0x13, 0x80F3);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x8B00);
	re_mdio_write(sc, 0x13, 0x80F0);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x3A00);
	re_mdio_write(sc, 0x13, 0x80EF);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x0500);
	re_mdio_write(sc, 0x13, 0x80F6);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x6E00);
	re_mdio_write(sc, 0x13, 0x80EC);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x6800);
	re_mdio_write(sc, 0x13, 0x80ED);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x7C00);
	re_mdio_write(sc, 0x13, 0x80F2);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0xF400);
	re_mdio_write(sc, 0x13, 0x80F4);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x8500);

	re_mdio_write(sc, 0x1F, 0x0A43);
	re_mdio_write(sc, 0x13, 0x8110);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0xA800);
	re_mdio_write(sc, 0x13, 0x810F);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x1D00);
	re_mdio_write(sc, 0x13, 0x8111);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0xF500);
	re_mdio_write(sc, 0x13, 0x8113);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x6100);
	re_mdio_write(sc, 0x13, 0x8115);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x9200);
	re_mdio_write(sc, 0x13, 0x810E);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x0400);
	re_mdio_write(sc, 0x13, 0x810C);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x7C00);
	re_mdio_write(sc, 0x13, 0x810B);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x5A00);

	re_mdio_write(sc, 0x1F, 0x0A43);
	re_mdio_write(sc, 0x13, 0x80D1);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0xFF00);
	re_mdio_write(sc, 0x13, 0x80CD);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x9E00);
	re_mdio_write(sc, 0x13, 0x80D3);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x0E00);
	re_mdio_write(sc, 0x13, 0x80D5);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0xCA00);
	re_mdio_write(sc, 0x13, 0x80D7);
	re_clear_set_eth_phy_bit(sc, 0x14, 0xFF00, 0x8400);

	if (phy_power_saving == 1) {
		re_mdio_write(sc, 0x1F, 0x0A43);
		re_set_eth_phy_bit(sc, 0x10, BIT_2);
		re_mdio_write(sc, 0x1F, 0x0000);
	} else {
		re_mdio_write(sc, 0x1F, 0x0A43);
		re_clear_eth_phy_bit(sc, 0x10, BIT_2);
		re_mdio_write(sc, 0x1F, 0x0000);
		DELAY(20000);
	}

	re_mdio_write(sc, 0x1F, 0x0A43);
	re_mdio_write(sc, 0x13, 0x8011);
	re_clear_eth_phy_bit(sc, 0x14, BIT_14);
	re_mdio_write(sc, 0x1F, 0x0A40);
	re_mdio_write(sc, 0x1F, 0x0000);
	re_mdio_write(sc, 0x00, 0x9200);
}

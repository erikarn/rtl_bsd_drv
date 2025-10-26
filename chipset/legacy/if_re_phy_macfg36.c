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

#include "if_re_phy_macfg36.h"

void
re_hw_phy_config_macfg36(struct re_softc *sc, bool phy_power_saving)
{
	uint16_t Data;

	re_mdio_write(sc, 0x1F, 0x0007);
	re_mdio_write(sc, 0x1E, 0x0023);
	Data = re_mdio_read(sc, 0x17) | 0x0006;
	if (sc->RequiredSecLanDonglePatch)
		Data &= ~(BIT_2);
	else
		Data |= (BIT_2);
	re_mdio_write(sc, 0x17, Data);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1f, 0x0005);
	re_mdio_write(sc, 0x05, 0x8b80);
	re_mdio_write(sc, 0x06, 0xc896);
	re_mdio_write(sc, 0x1f, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0001);
	re_mdio_write(sc, 0x0B, 0x6C20);
	re_mdio_write(sc, 0x07, 0x2872);
	re_mdio_write(sc, 0x1C, 0xEFFF);
	re_mdio_write(sc, 0x1F, 0x0003);
	re_mdio_write(sc, 0x14, 0x6420);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0002);
	Data = re_mdio_read(sc, 0x08) & 0x00FF;
	re_mdio_write(sc, 0x08, Data | 0x8000);

	re_mdio_write(sc, 0x1F, 0x0007);
	re_mdio_write(sc, 0x1E, 0x002D);
	Data = re_mdio_read(sc, 0x18);
	re_mdio_write(sc, 0x18, Data | 0x0050);
	re_mdio_write(sc, 0x1F, 0x0000);
	Data = re_mdio_read(sc, 0x14);
	re_mdio_write(sc, 0x14, Data | 0x8000);

	re_mdio_write(sc, 0x1F, 0x0002);
	re_mdio_write(sc, 0x00, 0x080B);
	re_mdio_write(sc, 0x0B, 0x09D7);
	re_mdio_write(sc, 0x1f, 0x0000);
	re_mdio_write(sc, 0x15, 0x1006);

	re_mdio_write(sc, 0x1F, 0x0007);
	re_mdio_write(sc, 0x1E, 0x002F);
	re_mdio_write(sc, 0x15, 0x1919);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0003);
	re_mdio_write(sc, 0x19, 0x7F46);
	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8AD2);
	re_mdio_write(sc, 0x06, 0x6810);
	re_mdio_write(sc, 0x05, 0x8AD4);
	re_mdio_write(sc, 0x06, 0x8002);
	re_mdio_write(sc, 0x05, 0x8ADE);
	re_mdio_write(sc, 0x06, 0x8025);
	re_mdio_write(sc, 0x1F, 0x0000);
}

void
re_hw_phy_disable_eee_macfg36(struct re_softc *sc)
{
	uint16_t data;

	re_mdio_write(sc, 0x1F, 0x0007);
	re_mdio_write(sc, 0x1E, 0x0020);
	data = re_mdio_read(sc, 0x15) & ~0x0100;
	re_mdio_write(sc, 0x15, data);
	re_mdio_write(sc, 0x1F, 0x0006);
	re_mdio_write(sc, 0x00, 0x5A00);
	re_mdio_write(sc, 0x1F, 0x0000);
	re_mdio_write(sc, 0x0D, 0x0007);
	re_mdio_write(sc, 0x0E, 0x003C);
	re_mdio_write(sc, 0x0D, 0x4007);
	re_mdio_write(sc, 0x0E, 0x0000);
	re_mdio_write(sc, 0x0D, 0x0000);
	re_mdio_write(sc, 0x1F, 0x0000);
	if (CSR_READ_1(sc, RE_CFG4) & RL_CFG4_CUSTOMIZED_LED) {
		re_mdio_write(sc, 0x1F, 0x0005);
		re_mdio_write(sc, 0x05, 0x8B82);
		data = re_mdio_read(sc, 0x06) & ~0x0010;
		re_mdio_write(sc, 0x05, 0x8B82);
		re_mdio_write(sc, 0x06, data);
		re_mdio_write(sc, 0x1F, 0x0000);
	}
}

void
re_hw_phy_enable_eee_macfg36(struct re_softc *sc)
{
	uint16_t data;

	re_mdio_write(sc, 0x1F, 0x0007);
	re_mdio_write(sc, 0x1E, 0x0020);
	data = re_mdio_read(sc, 0x15) | 0x0100;
	re_mdio_write(sc, 0x15, data);
	re_mdio_write(sc, 0x1F, 0x0006);
	re_mdio_write(sc, 0x00, 0x5A30);
	re_mdio_write(sc, 0x1F, 0x0000);
	re_mdio_write(sc, 0x0D, 0x0007);
	re_mdio_write(sc, 0x0E, 0x003C);
	re_mdio_write(sc, 0x0D, 0x4007);
	re_mdio_write(sc, 0x0E, 0x0006);
	re_mdio_write(sc, 0x0D, 0x0000);
	if ((CSR_READ_1(sc, RE_CFG4)&RL_CFG4_CUSTOMIZED_LED) &&
	    (CSR_READ_1(sc, RE_MACDBG) & BIT_7)) {
		re_mdio_write(sc, 0x1F, 0x0005);
		re_mdio_write(sc, 0x05, 0x8AC8);
		re_mdio_write(sc, 0x06, CSR_READ_1(sc, RE_CUSTOM_LED));
		re_mdio_write(sc, 0x05, 0x8B82);
		data = re_mdio_read(sc, 0x06) | 0x0010;
		re_mdio_write(sc, 0x05, 0x8B82);
		re_mdio_write(sc, 0x06, data);
		re_mdio_write(sc, 0x1F, 0x0000);
	}
}

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

#include "if_re_phy_macfg52.h"

void
re_hw_phy_config_macfg52(struct re_softc *sc, bool phy_power_saving)
{
	uint32_t Data_u32;
	uint16_t Data;

	Data_u32 = re_eri_read(sc, 0x1D0, 4, ERIAR_ExGMAC);
	Data_u32 &= ~BIT_1;
	re_eri_write(sc, 0x1D0, 4, Data_u32, ERIAR_ExGMAC);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8B80);
	Data = re_mdio_read(sc, 0x06);
	Data |= BIT_2 | BIT_1;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x1f, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0007);
	re_mdio_write(sc, 0x1E, 0x002D);
	Data = re_mdio_read(sc, 0x18);
	Data |= BIT_4;
	re_mdio_write(sc, 0x18, Data);
	re_mdio_write(sc, 0x1f, 0x0002);
	re_mdio_write(sc, 0x1f, 0x0000);
	Data = re_mdio_read(sc, 0x14);
	Data |= BIT_15;
	re_mdio_write(sc, 0x14, Data);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8B86);
	Data = re_mdio_read(sc, 0x06);
	Data |= BIT_0;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x1f, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8B85);
	Data = re_mdio_read(sc, 0x06);
	Data |= BIT_14;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x1f, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0003);
	re_mdio_write(sc, 0x09, 0xA20F);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8B55);
	re_mdio_write(sc, 0x06, 0x0000);
	re_mdio_write(sc, 0x05, 0x8B5E);
	re_mdio_write(sc, 0x06, 0x0000);
	re_mdio_write(sc, 0x05, 0x8B67);
	re_mdio_write(sc, 0x06, 0x0000);
	re_mdio_write(sc, 0x05, 0x8B70);
	re_mdio_write(sc, 0x06, 0x0000);
	re_mdio_write(sc, 0x1f, 0x0000);
	re_mdio_write(sc, 0x1F, 0x0007);
	re_mdio_write(sc, 0x1E, 0x0078);
	re_mdio_write(sc, 0x17, 0x0000);
	re_mdio_write(sc, 0x19, 0x00FB);
	re_mdio_write(sc, 0x1f, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8B79);
	re_mdio_write(sc, 0x06, 0xAA00);
	re_mdio_write(sc, 0x1f, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0003);
	re_mdio_write(sc, 0x01, 0x328A);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8B54);
	Data = re_mdio_read(sc, 0x06);
	Data &= ~BIT_11;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x05, 0x8B5D);
	Data = re_mdio_read(sc, 0x06);
	Data &= ~BIT_11;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x05, 0x8A7C);
	Data = re_mdio_read(sc, 0x06);
	Data &= ~BIT_8;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x05, 0x8A7F);
	Data = re_mdio_read(sc, 0x06);
	Data |= BIT_8;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x05, 0x8A82);
	Data = re_mdio_read(sc, 0x06);
	Data &= ~BIT_8;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x05, 0x8A85);
	Data = re_mdio_read(sc, 0x06);
	Data &= ~BIT_8;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x05, 0x8A88);
	Data = re_mdio_read(sc, 0x06);
	Data &= ~BIT_8;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x1F, 0x0000);

	re_mdio_write(sc, 0x1F, 0x0005);
	re_mdio_write(sc, 0x05, 0x8B85);
	Data = re_mdio_read(sc, 0x06);
	Data |= BIT_15;
	re_mdio_write(sc, 0x06, Data);
	re_mdio_write(sc, 0x1f, 0x0000);

	re_mdio_write(sc, 0x1f, 0x0003);
	Data = re_mdio_read(sc, 0x19);
	Data &= ~BIT_0;
	re_mdio_write(sc, 0x19, Data);
	Data = re_mdio_read(sc, 0x10);
	Data &= ~BIT_10;
	re_mdio_write(sc, 0x10, Data);
	re_mdio_write(sc, 0x1f, 0x0000);
}

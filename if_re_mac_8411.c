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

#include "if_re_ocp.h"
#include "if_re_mac_mcu.h"
#include "if_re_mac_8411.h"

static const uint16_t mcu_patch_code_8411b_1[] = {
	0xE008, 0xE00A, 0xE00C, 0xE00E, 0xE027, 0xE04F, 0xE05E, 0xE065, 0xC602,
	0xBE00, 0x0000, 0xC502, 0xBD00, 0x074C, 0xC302, 0xBB00, 0x080A, 0x6420,
	0x48C2, 0x8C20, 0xC516, 0x64A4, 0x49C0, 0xF009, 0x74A2, 0x8CA5, 0x74A0,
	0xC50E, 0x9CA2, 0x1C11, 0x9CA0, 0xE006, 0x74F8, 0x48C4, 0x8CF8, 0xC404,
	0xBC00, 0xC403, 0xBC00, 0x0BF2, 0x0C0A, 0xE434, 0xD3C0, 0x49D9, 0xF01F,
	0xC526, 0x64A5, 0x1400, 0xF007, 0x0C01, 0x8CA5, 0x1C15, 0xC51B, 0x9CA0,
	0xE013, 0xC519, 0x74A0, 0x48C4, 0x8CA0, 0xC516, 0x74A4, 0x48C8, 0x48CA,
	0x9CA4, 0xC512, 0x1B00, 0x9BA0, 0x1B1C, 0x483F, 0x9BA2, 0x1B04, 0xC508,
	0x9BA0, 0xC505, 0xBD00, 0xC502, 0xBD00, 0x0300, 0x051E, 0xE434, 0xE018,
	0xE092, 0xDE20, 0xD3C0, 0xC50F, 0x76A4, 0x49E3, 0xF007, 0x49C0, 0xF103,
	0xC607, 0xBE00, 0xC606, 0xBE00, 0xC602, 0xBE00, 0x0C4C, 0x0C28, 0x0C2C,
	0xDC00, 0xC707, 0x1D00, 0x8DE2, 0x48C1, 0xC502, 0xBD00, 0x00AA, 0xE0C0,
	0xC502, 0xBD00, 0x0132
};


void
re_set_mac_mcu_8411b_1(struct re_softc *sc)
{

	re_disable_mcu_bps(sc);

	re_write_mac_mcu_ram_code(sc, mcu_patch_code_8411b_1,
	    ARRAY_SIZE(mcu_patch_code_8411b_1));

	re_mac_ocp_write(sc, 0xFC26, 0x8000);

	re_mac_ocp_write(sc, 0xFC2A, 0x0743);
	re_mac_ocp_write(sc, 0xFC2C, 0x0801);
	re_mac_ocp_write(sc, 0xFC2E, 0x0BE9);
	re_mac_ocp_write(sc, 0xFC30, 0x02FD);
	re_mac_ocp_write(sc, 0xFC32, 0x0C25);
	re_mac_ocp_write(sc, 0xFC34, 0x00A9);
	re_mac_ocp_write(sc, 0xFC36, 0x012D);
}

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

#include "if_re_version.h"

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

#include <vm/vm.h>	      /* for vtophys */
#include <vm/pmap.h>	    /* for vtophys */
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
#include "if_re_eri.h"
#include "if_re_dash.h"

u_int32_t
re_get_dash_fw_ver(struct re_softc *sc)
{
	if (HW_DASH_SUPPORT_GET_FIRMWARE_VERSION(sc))
		return re_ocp_read(sc, OCP_REG_FIRMWARE_MAJOR_VERSION, 4);
	else
		return 0xffffffff;
}

static int
_re_check_dash(struct re_softc *sc)
{
	if (HW_DASH_SUPPORT_DASH(sc) == FALSE)
		return 0;

	if (!sc->AllowAccessDashOcp)
		return 0;

	if (HW_DASH_SUPPORT_TYPE_2(sc) || HW_DASH_SUPPORT_TYPE_3(sc) ||
	    HW_DASH_SUPPORT_TYPE_4(sc)) {
		return !!(re_ocp_read(sc, 0x128, 1) & BIT_0);
	} else if (HW_DASH_SUPPORT_TYPE_1(sc)) {
		if (sc->re_type == MACFG_66)
			return !!(re_ocp_read(sc, 0xb8, 2) & BIT_15);
		else
			return !!(re_ocp_read(sc, 0x10, 2) & BIT_15);
	}

	return 0;
}

int
re_check_dash(struct re_softc *sc)
{
	if (_re_check_dash(sc)) {
		u_int32_t ver = re_get_dash_fw_ver(sc);
		sc->re_dash_fw_ver = ver;
		if (!(ver == 0 || ver == 0xffffffff))
			return 1;
	}

	return 0;
}

void
re_wait_dash_fw_ready(struct re_softc *sc)
{
	int timeout;

	if (!HW_DASH_SUPPORT_DASH(sc))
		return;

	if (!sc->re_dash)
		return;

	if (HW_DASH_SUPPORT_TYPE_2(sc) || HW_DASH_SUPPORT_TYPE_3(sc) ||
	    HW_DASH_SUPPORT_TYPE_4(sc)) {
		for (timeout = 0; timeout < 10; timeout++) {
			DELAY(10000);
			if (re_ocp_read(sc, 0x124, 1) & BIT_0)
				break;
		}
	} else {
		u_int32_t reg;

		if (sc->re_type == MACFG_66)
			reg = 0xB8;
		else
			reg = 0x10;

		for (timeout = 0; timeout < 10; timeout++) {
			DELAY(10000);
			if (re_ocp_read(sc, reg, 2) & BIT_11)
				break;
		}
	}
}

void
re_notify_dash_oob_dp(struct re_softc *sc, u_int32_t cmd)
{
	if (!HW_DASH_SUPPORT_TYPE_1(sc))
		return;

	if (sc->re_type == MACFG_66 && cmd == OOB_CMD_DRIVER_START)
		CSR_WRITE_1(sc, RE_TwiCmdReg, CSR_READ_1(sc, RE_TwiCmdReg) | BIT_7);

	re_eri_write(sc, 0xE8, 1, (u_int8_t)cmd, ERIAR_ExGMAC);

	re_ocp_write(sc, 0x30, 1, 0x01);

	if (sc->re_type == MACFG_66 && cmd == OOB_CMD_DRIVER_STOP)
		CSR_WRITE_1(sc, RE_TwiCmdReg, CSR_READ_1(sc, RE_TwiCmdReg) & ~BIT_7);
}

void
re_notify_dash_oob_cmac(struct re_softc *sc, u_int32_t cmd)
{
	if (!HW_DASH_SUPPORT_CMAC(sc))
		return;

	re_ocp_write(sc, 0x180, 1, cmd);
	re_ocp_write(sc, 0x30, 1, re_ocp_read(sc, 0x30, 1) | BIT_0);
}

void
re_notify_dash_oob_ipc2(struct re_softc *sc, u_int32_t cmd)
{
	if (!HW_DASH_SUPPORT_IPC2(sc))
		return;

	re_ocp_write(sc, RE_IB2SOC_DATA, 4, cmd);
	re_ocp_write(sc, RE_IB2SOC_CMD, 4, 0x00);
	re_ocp_write(sc, RE_IB2SOC_SET, 4, 0x01);
}

void
re_notify_dash_oob(struct re_softc *sc, u_int32_t cmd)
{
	switch (sc->HwSuppDashVer) {
	case 1:
		return re_notify_dash_oob_dp(sc, cmd);
	case 2:
	case 3:
		return re_notify_dash_oob_cmac(sc, cmd);
	case 4:
		return re_notify_dash_oob_ipc2(sc, cmd);
	default:
		return;
	}
}

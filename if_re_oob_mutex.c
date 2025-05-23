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

#include "if_re_version.h"

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
#include <net/if_private.h>
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
#include "if_re_oob_mutex.h"

void
if_re_OOB_mutex_lock(struct re_softc *sc)
{
	u_int8_t reg_16, reg_a0;
	u_int32_t wait_cnt_0, wait_Cnt_1;
	u_int16_t ocp_reg_mutex_ib;
	u_int16_t ocp_reg_mutex_oob;
	u_int16_t ocp_reg_mutex_prio;

	if (!sc->re_dash)
		return;

	switch (sc->re_type) {
	case MACFG_63:
	case MACFG_64:
	case MACFG_65:
		ocp_reg_mutex_oob = 0x16;
		ocp_reg_mutex_ib = 0x17;
		ocp_reg_mutex_prio = 0x9C;
		break;
	case MACFG_66:
		ocp_reg_mutex_oob = 0x06;
		ocp_reg_mutex_ib = 0x07;
		ocp_reg_mutex_prio = 0x9C;
		break;
	case MACFG_61:
	case MACFG_62:
	case MACFG_67:
	case MACFG_70:
	case MACFG_71:
	case MACFG_72:
	case MACFG_73:
	case MACFG_80:
	case MACFG_81:
	case MACFG_84:
	case MACFG_85:
		ocp_reg_mutex_oob = 0x110;
		ocp_reg_mutex_ib = 0x114;
		ocp_reg_mutex_prio = 0x11C;
		break;
	default:
		return;
	}

	re_ocp_write(sc, ocp_reg_mutex_ib, 1, BIT_0);
	reg_16 = re_ocp_read(sc, ocp_reg_mutex_oob, 1);
	wait_cnt_0 = 0;
	while(reg_16) {
		reg_a0 = re_ocp_read(sc, ocp_reg_mutex_prio, 1);
		if (reg_a0) {
			re_ocp_write(sc, ocp_reg_mutex_ib, 1, 0x00);
			reg_a0 = re_ocp_read(sc, ocp_reg_mutex_prio, 1);
			wait_Cnt_1 = 0;
			while(reg_a0) {
				reg_a0 = re_ocp_read(sc, ocp_reg_mutex_prio, 1);

				wait_Cnt_1++;

				if (wait_Cnt_1 > 2000)
					break;
			};
			re_ocp_write(sc, ocp_reg_mutex_ib, 1, BIT_0);

		}
		reg_16 = re_ocp_read(sc, ocp_reg_mutex_oob, 1);

		wait_cnt_0++;

		if (wait_cnt_0 > 2000)
			break;
	};
}

void
if_re_OOB_mutex_unlock(struct re_softc *sc)
{
	u_int16_t ocp_reg_mutex_ib;
	u_int16_t ocp_reg_mutex_prio;

	if (!sc->re_dash)
		return;

	switch (sc->re_type) {
	case MACFG_63:
	case MACFG_64:
	case MACFG_65:
		ocp_reg_mutex_ib = 0x17;
		ocp_reg_mutex_prio = 0x9C;
		break;
	case MACFG_66:
		ocp_reg_mutex_ib = 0x07;
		ocp_reg_mutex_prio = 0x9C;
		break;
	case MACFG_61:
	case MACFG_62:
	case MACFG_67:
	case MACFG_70:
	case MACFG_71:
	case MACFG_72:
	case MACFG_73:
	case MACFG_80:
	case MACFG_81:
	case MACFG_84:
	case MACFG_85:
		ocp_reg_mutex_ib = 0x114;
		ocp_reg_mutex_prio = 0x11C;
		break;
	default:
		return;
	}

	re_ocp_write(sc, ocp_reg_mutex_prio, 1, BIT_0);
	re_ocp_write(sc, ocp_reg_mutex_ib, 1, 0x00);
}


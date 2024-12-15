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

#include "if_re_misc.h"
#include "if_re_ocp.h"
#include "if_re_mdio.h"
#include "if_re_hw_8125.h"

#define MIN_IPV4_PATCH_PKT_LEN (121)
#define MIN_IPV6_PATCH_PKT_LEN (147)

int
re_8125_pad(struct re_softc *sc,struct mbuf *m_head)
{
	uint32_t min_pkt_len;
	uint16_t ether_type;

	if ((m_head->m_pkthdr.csum_flags & (CSUM_TCP | CSUM_UDP)) != 0)
		goto out;

	ether_type = re_get_eth_type(m_head);
	min_pkt_len = RE_MIN_FRAMELEN;
	if (ether_type == ETHERTYPE_IP) {
		struct ip *ip = (struct ip *)mtodo(m_head, ETHER_HDR_LEN);
		if (ip->ip_p == IPPROTO_UDP)
			min_pkt_len = MIN_IPV4_PATCH_PKT_LEN;
	} else if (ether_type == ETHERTYPE_IPV6) {
		struct ip6_hdr *ip6 = (struct ip6_hdr *)mtodo(m_head, ETHER_HDR_LEN);
		if (ip6->ip6_nxt == IPPROTO_UDP)
			min_pkt_len = MIN_IPV6_PATCH_PKT_LEN;
	}

	if (m_head->m_pkthdr.len < min_pkt_len) {
		static const uint8_t pad[MIN_IPV4_PATCH_PKT_LEN];
		uint16_t pad_len = min_pkt_len - m_head->m_pkthdr.len;
		if (!m_append(m_head, pad_len, pad))
			return (1);

		if (ether_type == ETHERTYPE_IP &&
		    m_head->m_pkthdr.csum_flags & CSUM_IP) {
			struct ip *ip;
			m_head->m_data += ETHER_HDR_LEN;
			ip = mtod(m_head, struct ip *);
			ip->ip_sum = in_cksum(m_head, ip->ip_hl << 2);
			m_head->m_data -= ETHER_HDR_LEN;
			m_head->m_pkthdr.csum_flags &= ~CSUM_IP;
		}
	}

out:
	return(0);
}


int
re_ifmedia_upd_8125(struct ifnet *ifp)
{
	struct re_softc	*sc = ifp->if_softc;
	struct ifmedia	*ifm = &sc->media;
	int anar;
	int gbcr;
	int cr2500;

	if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
		return(EINVAL);

	//Disable Giga Lite
	re_clear_eth_ocp_phy_bit(sc, 0xA428, BIT_9);
	re_clear_eth_ocp_phy_bit(sc, 0xA5EA, BIT_0);
	if (sc->re_device_id == RT_DEVICEID_8126)
		re_clear_eth_ocp_phy_bit(sc, 0xB5EA, BIT_1);

	cr2500 = re_real_ocp_phy_read(sc, 0xA5D4) &
		 ~(RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL);
	gbcr = re_mdio_read(sc, MII_100T2CR) &
	       ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
	anar = re_mdio_read(sc, MII_ANAR) &
	       ~(ANAR_10 | ANAR_10_FD | ANAR_TX | ANAR_TX_FD | ANAR_FC | ANAR_PAUSE_ASYM);

	switch (IFM_SUBTYPE(ifm->ifm_media)) {
	case IFM_AUTO:
	case IFM_5000_T:
		if (sc->re_device_id == RT_DEVICEID_8126)
			cr2500 |= RTK_ADVERTISE_5000FULL;
	/*	FALLTHROUGH */
	case IFM_2500_SX:
	case IFM_2500_X:
	case IFM_2500_T:
		cr2500 |= RTK_ADVERTISE_2500FULL;
	/*	FALLTHROUGH */
	case MACFG_80:
	case IFM_1000_SX:
#if OS_VER < 500000
	case IFM_1000_TX:
#else
	case IFM_1000_T:
#endif
		gbcr |= GTCR_ADV_1000TFDX;
		anar |= ANAR_TX_FD;
	/*	FALLTHROUGH */
	case IFM_100_TX:
		anar |= ANAR_TX | ANAR_10_FD;
		if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX)
			anar |= ANAR_TX_FD;
	/*	FALLTHROUGH */
	case IFM_10_T:
		anar |= ANAR_10;
		if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX)
			anar |= ANAR_10_FD;

		if (sc->re_type == MACFG_13) {
			re_mdio_write(sc, MII_BMCR, 0x8000);
		}
		break;
	default:
		printf("re%d: Unsupported media type\n", sc->re_unit);
		return(0);
	}

	re_mdio_write(sc, 0x1F, 0x0000);
	re_real_ocp_phy_write(sc, 0xA5D4, cr2500);
	re_mdio_write(sc, MII_ANAR, anar | ANAR_FC | ANAR_PAUSE_ASYM);
	re_mdio_write(sc, MII_100T2CR, gbcr);
	re_mdio_write(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN | BMCR_STARTNEG);

	return(0);
}


void
re_ifmedia_sts_8125(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct re_softc		*sc;

	sc = ifp->if_softc;

	RE_LOCK(sc);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	if (re_link_ok(sc)) {
		u_int32_t msr;

		ifmr->ifm_status |= IFM_ACTIVE;

		msr = CSR_READ_4(sc, RE_PHY_STATUS);

		if (msr & RL_PHY_STATUS_FULL_DUP)
			ifmr->ifm_active |= IFM_FDX;
		else
			ifmr->ifm_active |= IFM_HDX;

		if (msr & RL_PHY_STATUS_10M)
			ifmr->ifm_active |= IFM_10_T;
		else if (msr & RL_PHY_STATUS_100M)
			ifmr->ifm_active |= IFM_100_TX;
		else if (msr & RL_PHY_STATUS_1000MF)
			ifmr->ifm_active |= IFM_1000_T;
		else if (msr & RL_PHY_STATUS_500MF)
			ifmr->ifm_active |= IFM_1000_T;
		else if (msr & RL_PHY_STATUS_1250MF)
			ifmr->ifm_active |= IFM_1000_T;
		else if (msr & RL_PHY_STATUS_2500MF)
			ifmr->ifm_active |= IFM_2500_T;
		else if (msr & RL_PHY_STATUS_5000MF_LITE)
			ifmr->ifm_active |= IFM_2500_T;
		else if (msr & RL_PHY_STATUS_5000MF)
			ifmr->ifm_active |= IFM_5000_T;
	}

	RE_UNLOCK(sc);

	return;
}


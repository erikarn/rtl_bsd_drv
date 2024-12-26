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

#include "if_re_hwrev.h"

#include "if_re_misc.h"

#include "if_re_eeprom.h"
#include "if_re_mdio.h"
#include "if_re_eri.h"
#include "if_re_efuse.h"
#include "if_re_ocp.h"
#include "if_re_cfg.h"
#include "if_re_csi.h"
#include "if_re_dash.h"
#include "if_re_mac_mcu.h"
#include "if_re_phy_mcu.h"

#include "if_re_mac_8168.h"
#include "if_re_phy_8168.h"

#include "if_re_phy_8169.h"

#include "if_re_mac_8125.h"
#include "if_re_hw_8125.h"
#include "if_re_phy_8125.h"

#include "if_re_mac_8126.h"
#include "if_re_phy_8126.h"

#include "if_re_mac_8411.h"
#include "if_re_phy_8411.h"

#include "if_re_phy_macfg27.h"
#include "if_re_phy_macfg28.h"
#include "if_re_phy_macfg31.h"
#include "if_re_phy_macfg32.h"
#include "if_re_phy_macfg33.h"
#include "if_re_phy_macfg36.h"
#include "if_re_phy_macfg38.h"
#include "if_re_phy_macfg39.h"
#include "if_re_phy_macfg41.h"
#include "if_re_phy_macfg42.h"
#include "if_re_phy_macfg50.h"
#include "if_re_phy_macfg51.h"
#include "if_re_phy_macfg52.h"
#include "if_re_phy_macfg53.h"
#include "if_re_phy_macfg54.h"
#include "if_re_phy_macfg56.h"
#include "if_re_phy_macfg58.h"
#include "if_re_phy_macfg59.h"
#include "if_re_phy_macfg60.h"
#include "if_re_phy_macfg61.h"
#include "if_re_phy_macfg62.h"
#include "if_re_phy_macfg63.h"
#include "if_re_phy_macfg64.h"
#include "if_re_phy_macfg65.h"
#include "if_re_phy_macfg66.h"

struct bus_dma_tag {
        struct bus_dma_tag_common common;
        int                     map_count;
        int                     bounce_flags;
        bus_dma_segment_t       *segments;
        struct bounce_zone      *bounce_zone;
};

/*
 * Various supported device vendors/types and their names.
 */
static struct re_type re_devs[] = {
        {
                RT_VENDORID, RT_DEVICEID_8169,
                "Realtek PCI GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_8169SC,
                "Realtek PCI GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_8168,
                "Realtek PCIe GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_8161,
                "Realtek PCIe GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_8162,
                "Realtek PCIe GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_8136,
                "Realtek PCIe FE Family Controller"
        },
        {
                DLINK_VENDORID, 0x4300,
                "Realtek PCI GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_8125,
                "Realtek PCIe 2.5GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_3000,
                "Killer PCIe 3x00 2.5GbE Family Controller"
        },
        {
                RT_VENDORID, RT_DEVICEID_8126,
                "Realtek PCIe 5GbE Family Controller"
        },
        { 0, 0, NULL }
};

static int	re_probe			__P((device_t));
static int	re_attach			__P((device_t));
static int	re_detach			__P((device_t));
static int	re_suspend 			__P((device_t));
static int	re_resume 			__P((device_t));
static int	re_shutdown			__P((device_t));

static void re_ephy_write			__P((struct re_softc*, u_int8_t, u_int16_t));
static u_int16_t re_ephy_read		__P((struct re_softc*, u_int8_t));

static void re_driver_start             __P((struct re_softc*));
static void re_driver_stop         	__P((struct re_softc*));

static void re_hw_phy_config		__P((struct re_softc *));
static void re_init			__P((void *));
static int  re_var_init			__P((struct re_softc *));
static void re_reset			__P((struct re_softc *));
static void re_stop			__P((struct re_softc *));
static void re_setwol			__P((struct re_softc *));
static void re_clrwol			__P((struct re_softc *));
static u_int8_t re_set_wol_linkspeed 	__P((struct re_softc *));

static void re_start				__P((struct ifnet *));
static void re_start_locked			__P((struct ifnet *));
static int re_encap				__P((struct re_softc *, struct mbuf **));
static void WritePacket				__P((struct re_softc *, bus_dma_segment_t*, int, int, uint32_t, uint32_t, uint32_t));
static void re_start_tx				__P((struct re_softc *));
static uint32_t CountFreeTxDescNum			__P((struct re_descriptor *));
//static int CountMbufNum				__P((struct mbuf *));
#ifdef RE_FIXUP_RX
static __inline void re_fixup_rx		__P((struct mbuf *));
#endif
static void re_txeof				__P((struct re_softc *));

//static int re_rxeof				__P((struct re_softc *));

#if OS_VER < VERSION(7,0)
static void re_intr				__P((void *));
#else
static int re_intr				__P((void *));
#endif //OS_VER < VERSION(7,0)
#if OS_VER < VERSION(7,0)
static void re_intr_8125				__P((void *));
#else
static int re_intr_8125				__P((void *));
#endif //OS_VER < VERSION(7,0)
//static void re_set_multicast_reg	__P((struct re_softc *, u_int32_t, u_int32_t));
static void re_clear_all_rx_packet_filter	__P((struct re_softc *));
static void re_set_rx_packet_filter_in_sleep_state	__P((struct re_softc *));
static void re_set_rx_packet_filter	__P((struct re_softc *));
static void re_setmulti			__P((struct re_softc *));
static int  re_ioctl			__P((struct ifnet *, u_long, caddr_t));
static void re_link_on_patch	__P((struct re_softc *));
static void re_link_down_patch	__P((struct re_softc *));
static void re_init_timer	__P((struct re_softc *));
static void re_stop_timer	__P((struct re_softc *));
static void re_start_timer	__P((struct re_softc *));
static void re_tick				__P((void *));
#if OS_VER < VERSION(7,0)
static void re_watchdog				__P((struct ifnet *));
#endif

static int  re_ifmedia_upd			__P((struct ifnet *));
static void re_ifmedia_sts			__P((struct ifnet *, struct ifmediareq *));

static void re_int_task_poll		(void *, int);
static void re_int_task				(void *, int);
static void re_int_task_8125_poll	(void *, int);
static void re_int_task_8125		(void *, int);

//static void re_phy_power_up(device_t dev);
//static void re_phy_power_down(device_t dev);
static int re_alloc_buf(struct re_softc *);
static void re_release_buf(struct re_softc *);
static void set_rxbufsize(struct re_softc*);
static void re_release_rx_buf(struct re_softc *);
static void re_release_tx_buf(struct re_softc *);
static void OOB_mutex_lock(struct re_softc *);
static void OOB_mutex_unlock(struct re_softc *);
static void re_hw_start_unlock(struct re_softc *sc);
static void re_hw_start_unlock_8125(struct re_softc *sc);

static void re_add_sysctls      (struct re_softc *);
static int re_sysctl_driver_variable      (SYSCTL_HANDLER_ARGS);
static int re_sysctl_stats      (SYSCTL_HANDLER_ARGS);
static int re_sysctl_registers  (SYSCTL_HANDLER_ARGS);
static int re_sysctl_registers2  (SYSCTL_HANDLER_ARGS);
static int re_sysctl_registers3  (SYSCTL_HANDLER_ARGS);
static int re_sysctl_registers4  (SYSCTL_HANDLER_ARGS);
static int re_sysctl_registers5  (SYSCTL_HANDLER_ARGS);
static int re_sysctl_eth_phy    (SYSCTL_HANDLER_ARGS);
static int re_sysctl_dump_rx_desc    (SYSCTL_HANDLER_ARGS);
static int re_sysctl_dump_tx_desc    (SYSCTL_HANDLER_ARGS);
static int re_sysctl_pcie_phy   (SYSCTL_HANDLER_ARGS);
static int re_sysctl_extended_registers   (SYSCTL_HANDLER_ARGS);
static int re_sysctl_pci_registers        (SYSCTL_HANDLER_ARGS);
static int re_sysctl_msix_tbl             (SYSCTL_HANDLER_ARGS);

/* Tunables. */
SYSCTL_NODE(_hw, OID_AUTO, re, CTLFLAG_RW | CTLFLAG_MPSAFE, 0, "");
static int msi_disable = 1;
SYSCTL_INT(_hw_re, OID_AUTO, msi_disable, CTLFLAG_RDTUN, &msi_disable, 0,
    "");
static int msix_disable = 0;
SYSCTL_INT(_hw_re, OID_AUTO, msix_disable, CTLFLAG_RDTUN, &msix_disable, 0,
    "");
static int prefer_iomap = 0;
SYSCTL_INT(_hw_re, OID_AUTO, prefer_iomap, CTLFLAG_RDTUN, &prefer_iomap, 0,
    "");
static int re_lro_entry_count = 128;
SYSCTL_INT(_hw_re, OID_AUTO, lro_entry_count, CTLFLAG_RDTUN,
    &re_lro_entry_count, 0, "");
static int re_lro_mbufq_depth = RE_RX_BUF_NUM;
SYSCTL_INT(_hw_re, OID_AUTO, lro_mbufq_depth, CTLFLAG_RDTUN,
    &re_lro_mbufq_depth, 0, "");
#ifdef ENABLE_EEE
static int eee_enable = 1;
#else
static int eee_enable = 0;
#endif
SYSCTL_INT(_hw_re, OID_AUTO, eee_enable, CTLFLAG_RDTUN, &eee_enable, 0,
    "");
static int phy_power_saving = 1;
SYSCTL_INT(_hw_re, OID_AUTO, phy_power_saving, CTLFLAG_RDTUN,
    &phy_power_saving, 0, "");
static int phy_mdix_mode = RE_ETH_PHY_AUTO_MDI_MDIX;
SYSCTL_INT(_hw_re, OID_AUTO, phy_mdix_mode, CTLFLAG_RDTUN, &phy_mdix_mode,
    0, "");
#ifdef ENABLE_S5WOL
static int s5wol = 1;
#else
static int s5wol = 0;
SYSCTL_INT(_hw_re, OID_AUTO, s5wol, CTLFLAG_RDTUN, &s5wol, 0, "");
#endif
#ifdef ENABLE_S0_MAGIC_PACKET
static int s0_magic_packet = 1;
#else
static int s0_magic_packet = 0;
#endif
SYSCTL_INT(_hw_re, OID_AUTO, s0_magic_packet, CTLFLAG_RDTUN,
    &s0_magic_packet, 0, "");
#ifdef CONFIG_SOC_LAN
static int config_soc_lan = 1;
#else
static int config_soc_lan = 0;
#endif
SYSCTL_INT(_hw_re, OID_AUTO, config_soc_lan, CTLFLAG_RDTUN,
    &config_soc_lan, 0, "");
#ifdef ENABLE_INTERRUPT_MITIGATIN
static int interrupt_mitigation = 1;
#else
static int interrupt_mitigation = 0;
#endif
SYSCTL_INT(_hw_re, OID_AUTO, interrupt_mitigation, CTLFLAG_RDTUN,
    &interrupt_mitigation, 0, "");
static int max_rx_mbuf_sz = MJUM9BYTES;
SYSCTL_INT(_hw_re, OID_AUTO, max_rx_mbuf_sz, CTLFLAG_RDTUN,
    &max_rx_mbuf_sz, 0, "");

#define RE_CSUM_FEATURES_IPV4    (CSUM_IP | CSUM_TCP | CSUM_UDP)
#define RE_CSUM_FEATURES_IPV6    (CSUM_TCP_IPV6 | CSUM_UDP_IPV6)
#define RE_CSUM_FEATURES    (RE_CSUM_FEATURES_IPV4 | RE_CSUM_FEATURES_IPV6)

static device_method_t re_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe, re_probe),
        DEVMETHOD(device_attach, re_attach),
        DEVMETHOD(device_detach, re_detach),
        DEVMETHOD(device_suspend, re_suspend),
        DEVMETHOD(device_resume, re_resume),
        DEVMETHOD(device_shutdown, re_shutdown),
        { 0, 0 }
};

static driver_t re_driver = {
        "re",
        re_methods,
        sizeof(struct re_softc)
};

#if OS_VER>=VERSION(14,0)
DRIVER_MODULE(if_re, pci, re_driver, 0, 0);
#else
static devclass_t re_devclass;
DRIVER_MODULE(if_re, pci, re_driver, re_devclass, 0, 0);
#endif
MODULE_DEPEND(if_re, pci, 1, 1, 1);
MODULE_DEPEND(if_re, ether, 1, 1, 1);

static void re_clear_phy_ups_reg(struct re_softc *sc)
{
        switch(sc->re_type) {
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_clear_eth_ocp_phy_bit(sc, 0xA466, BIT_0);
        /*	FALLTHROUGH */
        case MACFG_80:
        case MACFG_81:
                re_clear_eth_ocp_phy_bit(sc, 0xA468, BIT_3 | BIT_1);
                break;
        };
}

static int re_is_ups_resume(struct re_softc *sc)
{
        switch(sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                return (re_mac_ocp_read(sc, 0xD42C) & BIT_8);
        default:
                return (re_mac_ocp_read(sc, 0xD408) & BIT_0);
        }
}

static void re_clear_ups_resume_bit(struct re_softc *sc)
{
        switch(sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_mac_ocp_write(sc, 0xD42C, re_mac_ocp_read(sc, 0xD42C) & ~(BIT_8));
                break;
        default:
                re_mac_ocp_write(sc, 0xD408, re_mac_ocp_read(sc, 0xD408) & ~(BIT_0));
                break;
        }
}

static u_int8_t re_get_phy_state(struct re_softc *sc)
{
        switch(sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                return (re_real_ocp_phy_read(sc, 0xA420) & 0x7);
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
                return (re_ocp_phy_read(sc, 0x0A42, 0x10) & 0x7);
        default:
                return 0xff;
        };
}

static void re_wait_phy_ups_resume(struct re_softc *sc, u_int16_t PhyState)
{
        switch(sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                for (int i=0; i< 100; i++) {
                        if (re_get_phy_state(sc) == PhyState)
                                break;
                        else
                                DELAY(1000);
                }
                break;
        default:
                break;
        };
}

static void re_phy_power_up(device_t dev)
{
        struct re_softc		*sc;
        u_int8_t Data8;

        sc = device_get_softc(dev);

        if ((sc->re_if_flags & RL_FLAG_PHYWAKE_PM) != 0)
                CSR_WRITE_1(sc, RE_PMCH, CSR_READ_1(sc, RE_PMCH) | (BIT_6|BIT_7));

        re_mdio_write(sc, 0x1F, 0x0000);

        switch (sc->re_type) {
        case MACFG_4:
        case MACFG_5:
        case MACFG_6:
        case MACFG_21:
        case MACFG_22:
        case MACFG_23:
        case MACFG_24:
        case MACFG_25:
        case MACFG_26:
        case MACFG_27:
        case MACFG_28:
        case MACFG_31:
        case MACFG_32:
        case MACFG_33:
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
                re_mdio_write(sc, 0x0e, 0x0000);
                break;
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_61:
                Data8 = re_eri_read(sc, 0x1AB, 1, ERIAR_ExGMAC);
                Data8 |= (BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7);
                re_eri_write(sc, 0x1AB, 1, Data8, ERIAR_ExGMAC);
                break;
        default:
                break;
        };


        re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN);

        //wait mdc/mdio ready
        switch(sc->re_type) {
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
                DELAY(10000);
                break;
        }

        //wait ups resume (phy state 3)
        switch(sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_wait_phy_ups_resume(sc, 3);
                break;
        };
}

static u_int16_t re_get_phy_lp_ability(struct re_softc *sc)
{
        u_int16_t anlpar;

        re_mdio_write(sc, 0x1F, 0x0000);
        anlpar = re_mdio_read(sc, MII_ANLPAR);

        return anlpar;
}

static void re_phy_power_down(device_t dev)
{
        struct re_softc		*sc;
        u_int8_t Data8;

        sc = device_get_softc(dev);

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(sc))
                return;
#endif //ENABLE_FIBER_SUPPORT

        if (sc->re_dash) {
                re_set_wol_linkspeed(sc);
                return;
        }

        re_mdio_write(sc, 0x1F, 0x0000);

        switch (sc->re_type) {
        case MACFG_21:
        case MACFG_22:
        case MACFG_23:
        case MACFG_24:
        case MACFG_25:
        case MACFG_26:
        case MACFG_27:
        case MACFG_28:
        case MACFG_31:
        case MACFG_32:
        case MACFG_33:
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
                re_mdio_write(sc, 0x0e, 0x0200);
                re_mdio_write(sc, MII_BMCR, (BMCR_AUTOEN|BMCR_PDOWN));
                break;
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_61:
                Data8 = re_eri_read(sc, 0x1AB, 1, ERIAR_ExGMAC);
                Data8 &= ~(BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7);
                re_eri_write(sc, 0x1AB, 1, Data8, ERIAR_ExGMAC);

                re_mdio_write(sc, MII_BMCR, (BMCR_AUTOEN|BMCR_PDOWN));
                break;
        default:
                re_mdio_write(sc, MII_BMCR, (BMCR_AUTOEN|BMCR_PDOWN));
                break;
        }

        switch (sc->re_type) {
        case MACFG_36:
        case MACFG_37:
        case MACFG_42:
        case MACFG_43:
        case MACFG_54:
        case MACFG_55:
                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) & ~BIT_6);
                break;
        case MACFG_38:
        case MACFG_39:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) & ~BIT_6);
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~BIT_6);
                break;
        }

        if ((sc->re_if_flags & RL_FLAG_PHYWAKE_PM) != 0)
                CSR_WRITE_1(sc, RE_PMCH, CSR_READ_1(sc, RE_PMCH) & ~(BIT_6|BIT_7));
}

/*
static void re_tx_dma_map_buf(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
        union TxDesc *txptr = arg;

        if (error) {
                txptr->ul[0] &= ~htole32(RL_TDESC_CMD_BUFLEN);
                txptr->so1.TxBuff = 0;
                return;
        }

        txptr->so1.TxBuff = htole64(segs->ds_addr);
}
*/

static void re_rx_dma_map_buf(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
        union RxDesc *rxptr = arg;

        if (error) {
                rxptr->ul[0] &= ~htole32(RL_RDESC_CMD_BUFLEN);
                /* make sure desc is releasing before change buffer address */
                wmb();
                rxptr->so0.RxBuff = 0;
                return;
        }

        rxptr->so0.RxBuff = htole64(segs->ds_addr);
}

static void re_dma_map_rxdesc(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
        struct re_softc *sc = arg;

        if (error)
                return;

        CSR_WRITE_4(sc, 0xe4, RL_ADDR_LO(segs->ds_addr));
        CSR_WRITE_4(sc, 0xe8, RL_ADDR_HI(segs->ds_addr));
}

static void re_dma_map_txdesc(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
        struct re_softc *sc = arg;

        if (error)
                return;

        CSR_WRITE_4(sc, 0x20, RL_ADDR_LO(segs->ds_addr));
        CSR_WRITE_4(sc, 0x24, RL_ADDR_HI(segs->ds_addr));
}

/*
 * Probe for a RealTek 8129/8139 chip. Check the PCI vendor and device
 * IDs against our list and return a device name if we find a match.
 */
static int re_probe(device_t dev)	/* Search for Realtek NIC chip */
{
        struct re_type		*t;
        t = re_devs;
        while (t->re_name != NULL) {
                if ((pci_get_vendor(dev) == t->re_vid) &&
                    (pci_get_device(dev) == t->re_did)) {
                        device_set_desc(dev, t->re_name);
                        return(0);
                }
                t++;
        }

        return(ENXIO);
}

static void
re_dma_map_addr(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
        bus_addr_t              *addr;

        if (error)
                return;

        KASSERT(nseg == 1, ("too many DMA segments, %d should be 1", nseg));
        addr = arg;
        *addr = segs->ds_addr;
}

static int re_alloc_stats(device_t dev, struct re_softc *sc)
{
        int                     error;

        /* Create DMA map for statistics. */
        error = bus_dma_tag_create(sc->re_parent_tag, RE_DUMP_ALIGN, 0,
                                   BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
                                   sizeof(struct re_stats), 1, sizeof(struct re_stats), 0, NULL, NULL,
                                   &sc->re_tally.re_stag);
        if (error) {
                device_printf(dev, "could not create statistics DMA tag\n");
                return (error);
        }
        /* Allocate DMA'able memory for statistics. */
        error = bus_dmamem_alloc(sc->re_tally.re_stag,
                                 (void **)&sc->re_tally.re_stats,
                                 BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO,
                                 &sc->re_tally.re_smap);
        if (error) {
                device_printf(dev,
                              "could not allocate statistics DMA memory\n");
                return (error);
        }
        /* Load the map for statistics. */
        sc->re_tally.re_stats_addr = 0;
        error = bus_dmamap_load(sc->re_tally.re_stag, sc->re_tally.re_smap,
                                sc->re_tally.re_stats, sizeof(struct re_stats), re_dma_map_addr,
                                &sc->re_tally.re_stats_addr, BUS_DMA_NOWAIT);
        if (error != 0 || sc->re_tally.re_stats_addr == 0) {
                device_printf(dev, "could not load statistics DMA memory\n");
                return (ENOMEM);
        }

        return (0);
}

static void re_release_rx_buf(struct re_softc *sc)
{
        int i;

        if (sc->re_desc.re_rx_mtag) {
                for (i = 0; i < RE_RX_BUF_NUM; i++) {
                        if (sc->re_desc.rx_buf[i]!=NULL) {
                                bus_dmamap_sync(sc->re_desc.re_rx_mtag,
                                                sc->re_desc.re_rx_dmamap[i],
                                                BUS_DMASYNC_POSTREAD);
                                bus_dmamap_unload(sc->re_desc.re_rx_mtag,
                                                  sc->re_desc.re_rx_dmamap[i]);
                                bus_dmamap_destroy(sc->re_desc.re_rx_mtag,
                                                   sc->re_desc.re_rx_dmamap[i]);
                                m_freem(sc->re_desc.rx_buf[i]);
                                sc->re_desc.rx_buf[i] =NULL;
                        }
                }
                bus_dma_tag_destroy(sc->re_desc.re_rx_mtag);
                sc->re_desc.re_rx_mtag =0;
        }

}

static void re_release_tx_buf(struct re_softc *sc)
{
        int i;

        if (sc->re_desc.re_tx_mtag) {
                for (i = 0; i < RE_TX_BUF_NUM; i++) {
                        bus_dmamap_destroy(sc->re_desc.re_tx_mtag,
                                           sc->re_desc.re_tx_dmamap[i]);
                        m_freem(sc->re_desc.tx_buf[i]);
                }
                bus_dma_tag_destroy(sc->re_desc.re_tx_mtag);
                sc->re_desc.re_tx_mtag = 0;
        }
}

static void
re_free_soft_lro(struct re_softc *sc)
{
#if defined(INET) || defined(INET6)
        if (sc->re_lro.ifp) {
                tcp_lro_free(&sc->re_lro);
                sc->re_lro.ifp = NULL;
        }
#endif
}

static int
re_config_soft_lro(struct re_softc *sc)
{
        struct lro_ctrl *lro;

        lro = &sc->re_lro;
        bzero(lro, sizeof(struct lro_ctrl));

#if defined(INET) || defined(INET6)
#if OS_VER >= VERSION(11,0)
        if (tcp_lro_init_args(lro, sc->re_ifp,
                              max(TCP_LRO_ENTRIES, re_lro_entry_count),
                              min(1024, re_lro_mbufq_depth)) != 0) {
                device_printf(sc->dev,
                              "%s: tcp_lro_init_args failed\n",
                              __func__);
                return (ENOMEM);
        }
#else
        if (tcp_lro_init(lro)) {
                device_printf(sc->dev,
                              "%s: tcp_lro_init failed\n",
                              __func__);
                return (-1);
        }
#endif //OS_VER >= VERSION(11,0)
#endif
        lro->ifp = sc->re_ifp;

        return (0);
}

static void re_release_buf(struct re_softc *sc)
{
        re_release_rx_buf(sc);
        re_release_tx_buf(sc);
}

static int re_alloc_buf(struct re_softc *sc)
{
        int error =0;
        int i,size;

        switch(sc->re_type) {
        case MACFG_3:
        case MACFG_4:
        case MACFG_5:
        case MACFG_6:
        case MACFG_11:
        case MACFG_12:
        case MACFG_13:
        case MACFG_21:
        case MACFG_22:
        case MACFG_23:
                size = RE_TX_MAXSIZE_32K;
                break;
        default:
                size = RE_TX_MAXSIZE_64K;
                break;
        }
        RE_UNLOCK(sc);
        error = bus_dma_tag_create(sc->re_parent_tag, 1, 0,
                                   BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL,
                                   NULL, size, RE_NTXSEGS, size, 0,
                                   NULL, NULL, &sc->re_desc.re_tx_mtag);

        if (error) {
                //device_printf(dev,"re_tx_mtag fail\n");
                //goto fail;
                RE_LOCK(sc);
                return error;
        }

        error = bus_dma_tag_create(
                        sc->re_parent_tag,
                        RE_RX_BUFFER_ALIGN, 0,		/* alignment, boundary */
                        BUS_SPACE_MAXADDR,		/* lowaddr */
                        BUS_SPACE_MAXADDR,		/* highaddr */
                        NULL, NULL,			/* filter, filterarg */
                        sc->re_rx_desc_buf_sz, 1,			/* maxsize,nsegments */
                        sc->re_rx_desc_buf_sz,			/* maxsegsize */
                        0,				/* flags */
                        NULL, NULL,			/* lockfunc, lockarg */
                        &sc->re_desc.re_rx_mtag);
        if (error) {
                //device_printf(dev,"re_rx_mtag fail\n");
                //goto fail;
                RE_LOCK(sc);
                return error;
        }

        RE_LOCK(sc);
        if (sc->re_rx_mbuf_sz <= MCLBYTES)
                size = MCLBYTES;
        else if (sc->re_rx_mbuf_sz <=  MJUMPAGESIZE)
                size = MJUMPAGESIZE;
        else
                size =MJUM9BYTES;
        for (i = 0; i < RE_RX_BUF_NUM; i++) {
                sc->re_desc.rx_buf[i] = m_getjcl(M_DONTWAIT, MT_DATA, M_PKTHDR, size);
                if (!sc->re_desc.rx_buf[i]) {
                        //device_printf(dev, "m_getcl fail!!!\n");
                        error = ENXIO;
                        //goto fail;
                        return error;
                }

                sc->re_desc.rx_buf[i]->m_len = sc->re_desc.rx_buf[i]->m_pkthdr.len = size;
#ifdef RE_FIXUP_RX
                /*
                 * This is part of an evil trick to deal with non-x86 platforms.
                 * The RealTek chip requires RX buffers to be aligned on 64-bit
                 * boundaries, but that will hose non-x86 machines. To get around
                 * this, we leave some empty space at the start of each buffer
                 * and for non-x86 hosts, we copy the buffer back six bytes
                 * to achieve word alignment. This is slightly more efficient
                 * than allocating a new buffer, copying the contents, and
                 * discarding the old buffer.
                 */
                m_adj(sc->re_desc.rx_buf[i], RE_ETHER_ALIGN);
#endif

                error = bus_dmamap_create(sc->re_desc.re_rx_mtag, BUS_DMA_NOWAIT, &sc->re_desc.re_rx_dmamap[i]);
                if (error) {
                        //device_printf(dev, "bus_dmamap_create fail!!!\n");
                        //goto fail;
                        return error;
                }
        }

        for (i = 0; i < RE_TX_BUF_NUM; i++) {
                error = bus_dmamap_create(sc->re_desc.re_tx_mtag, BUS_DMA_NOWAIT, &sc->re_desc.re_tx_dmamap[i]);
                if (error) {
                        //device_printf(dev, "bus_dmamap_create fail!!!\n");
                        //goto fail;
                        return error;
                }
        }

        return 0;
}

static void set_rxbufsize(struct re_softc *sc)
{
        struct ifnet		*ifp;
        ifp = RE_GET_IFNET(sc);
        sc->re_rx_desc_buf_sz = (ifp->if_mtu > ETHERMTU) ? ifp->if_mtu: ETHERMTU;
        sc->re_rx_desc_buf_sz += (ETHER_VLAN_ENCAP_LEN + ETHER_HDR_LEN + ETHER_CRC_LEN);
        if (!(sc->re_if_flags & RL_FLAG_8168G_PLUS) ||
            sc->re_type == MACFG_56 || sc->re_type == MACFG_57 ||
            sc->re_type == MACFG_58 || sc->re_type == MACFG_59 ||
            sc->re_type == MACFG_60)
                sc->re_rx_desc_buf_sz += 1;
        CSR_WRITE_2(sc, RE_RxMaxSize, sc->re_rx_desc_buf_sz);
}

static void re_enable_force_clkreq(struct re_softc *sc, bool enable)
{
        if (enable)
                CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1) | BIT_7);
        else
                CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1) & ~BIT_7);
}

static void _re_enable_aspm_clkreq_lock(struct re_softc *sc, bool enable)
{
        switch(sc->re_type) {
        case MACFG_91:
        case MACFG_92:
                if (enable) {
                        CSR_WRITE_1(sc, RE_INT_CFG0_8125, CSR_READ_1(sc, RE_INT_CFG0_8125) | BIT_3);
                        CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) | BIT_0);
                } else {
                        CSR_WRITE_1(sc, RE_INT_CFG0_8125, CSR_READ_1(sc, RE_INT_CFG0_8125) & ~BIT_3);
                        CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) & ~BIT_0);
                }
                break;
        default:
                if (enable) {
                        CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) | BIT_0);
                        CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_7);
                } else {
                        CSR_WRITE_1(sc, RE_CFG5, CSR_READ_1(sc, RE_CFG5) & ~BIT_0);
                        CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) & ~BIT_7);
                }
                break;
        }
}

static void re_hw_mac_mcu_config(struct re_softc *sc)
{
        switch(sc->re_type) {
        case MACFG_56:
                re_set_mac_mcu_8168g_1(sc);
                break;
        case MACFG_58:
                re_set_mac_mcu_8168gu_1(sc);
                break;
        case MACFG_59:
                re_set_mac_mcu_8168gu_2(sc);
                break;
        case MACFG_60:
                re_set_mac_mcu_8411b_1(sc);
                break;
        case MACFG_62:
                re_set_mac_mcu_8168ep_1(sc);
                break;
        case MACFG_67:
                re_set_mac_mcu_8168ep_2(sc);
                break;
        case MACFG_68:
                re_set_mac_mcu_8168h_1(sc);
                break;
        case MACFG_69:
        case MACFG_76:
                re_set_mac_mcu_8168h_2(sc);
                break;
        case MACFG_70:
                re_set_mac_mcu_8168fp_1(sc);
                break;
        case MACFG_71:
                re_set_mac_mcu_8168fp_2(sc);
                break;
        case MACFG_72:
                re_set_mac_mcu_8168fp_3(sc);
                break;
        case MACFG_73:
                re_set_mac_mcu_8168fp_4(sc);
                break;
        case MACFG_74:
                re_set_mac_mcu_8168h_3(sc);
                break;
        case MACFG_75:
                re_set_mac_mcu_8168h_4(sc);
                break;
        case MACFG_80:
                re_set_mac_mcu_8125a_1(sc);
                break;
        case MACFG_81:
                re_set_mac_mcu_8125a_2(sc);
                break;
        case MACFG_82:
                re_set_mac_mcu_8125b_1(sc);
                break;
        case MACFG_83:
                re_set_mac_mcu_8125b_2(sc);
                break;
        case MACFG_84:
                re_set_mac_mcu_8125bp_1(sc);
                break;
        case MACFG_85:
                re_set_mac_mcu_8125bp_2(sc);
                break;
        case MACFG_86:
                re_set_mac_mcu_8125d_1(sc);
                break;
        case MACFG_87:
                re_set_mac_mcu_8125d_2(sc);
                break;
        case MACFG_90:
                re_set_mac_mcu_8126a_1(sc);
                break;
        case MACFG_91:
                re_set_mac_mcu_8126a_2(sc);
                break;
        case MACFG_92:
                re_set_mac_mcu_8126a_3(sc);
                break;
        }
}

#define ISRIMR_DASH_TYPE2_TX_DISABLE_IDLE BIT_5
static void Dash2DisableTx(struct re_softc *sc)
{
        u_int16_t WaitCnt;
        u_int8_t TmpUchar;

        if (!HW_DASH_SUPPORT_CMAC(sc))
                return;

        //Disable oob Tx
        RE_CMAC_WRITE_1(sc, RE_CMAC_IBCR2, RE_CMAC_READ_1(sc, RE_CMAC_IBCR2) & ~(BIT_0));
        WaitCnt = 0;

        //wait oob tx disable
        do {
                TmpUchar = RE_CMAC_READ_1(sc, RE_CMAC_IBISR0);

                if (TmpUchar & ISRIMR_DASH_TYPE2_TX_DISABLE_IDLE) {
                        break;
                }

                DELAY(50);
                WaitCnt++;
        } while(WaitCnt < 2000);

        //Clear ISRIMR_DASH_TYPE2_TX_DISABLE_IDLE
        RE_CMAC_WRITE_1(sc, RE_CMAC_IBISR0, RE_CMAC_READ_1(sc, RE_CMAC_IBISR0) | ISRIMR_DASH_TYPE2_TX_DISABLE_IDLE);
}

static void Dash2DisableRx(struct re_softc *sc)
{
        if (!HW_DASH_SUPPORT_CMAC(sc))
                return;

        RE_CMAC_WRITE_1(sc, RE_CMAC_IBCR0, RE_CMAC_READ_1(sc, RE_CMAC_IBCR0) & ~(BIT_0));
}

static void Dash2DisableTxRx(struct re_softc *sc)
{
        if (!HW_DASH_SUPPORT_CMAC(sc))
                return;

        Dash2DisableTx(sc);
        Dash2DisableRx(sc);
}

static void re_disable_now_is_oob(struct re_softc *sc)
{
        if (sc->re_hw_supp_now_is_oob_ver == 1)
                CSR_WRITE_1(sc, RE_MCU_CMD, CSR_READ_1(sc, RE_MCU_CMD) & ~RE_NOW_IS_OOB);
}

static void re_switch_to_sgmii_mode(struct re_softc *sc)
{
        if (FALSE == HW_SUPP_SERDES_PHY(sc)) return;

        switch (sc->hw_hw_supp_serdes_phy_ver) {
        case 1:
                re_mac_ocp_write(sc, 0xEB00, 0x2);
                re_set_mac_ocp_bit(sc, 0xEB16, BIT_1);
                break;
        }
}

static void
re_enable_magic_packet(struct re_softc *sc)
{
        if (sc->re_if_flags & RL_FLAG_MAGIC_PACKET_V3)
                re_set_mac_ocp_bit(sc, 0xC0B6, BIT_0);
        else if (sc->re_if_flags & RL_FLAG_MAGIC_PACKET_V2)
                re_eri_write(sc, 0xDC, 4,
                             re_eri_read(sc, 0xDC, 4, ERIAR_ExGMAC) | BIT_16,
                             ERIAR_ExGMAC);
        else
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | RL_CFG3_WOL_MAGIC);
}

static void
re_disable_magic_packet(struct re_softc *sc)
{
        if (sc->re_if_flags & RL_FLAG_MAGIC_PACKET_V3)
                re_clear_mac_ocp_bit(sc, 0xC0B6, BIT_0);
        else if (sc->re_if_flags & RL_FLAG_MAGIC_PACKET_V2)
                re_eri_write(sc, 0xDC, 4,
                             re_eri_read(sc, 0xDC, 4, ERIAR_ExGMAC) & ~BIT_16,
                             ERIAR_ExGMAC);
        else
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~RL_CFG3_WOL_MAGIC);
}

static void re_exit_oob(struct re_softc *sc)
{
        u_int16_t data16;
        int i;

        re_disable_cfg9346_write(sc);

        if (HW_SUPP_SERDES_PHY(sc)) {
                if (sc->hw_hw_supp_serdes_phy_ver == 1) {
                        re_switch_to_sgmii_mode(sc);
                }
        }

        switch(sc->re_type) {
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_80:
        case MACFG_81:
                Dash2DisableTxRx(sc);
                break;
        }

        if (HW_DASH_SUPPORT_DASH(sc))
                re_driver_start(sc);

        switch(sc->re_type) {
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_3);
                DELAY(2000);

                for (i = 0; i < 3000; i++) {
                        DELAY(50);
                        if (CSR_READ_4(sc, RE_TXCFG) & BIT_11)
                                break;
                }

                if (CSR_READ_1(sc, RE_COMMAND) & (RE_CMD_TX_ENB | RE_CMD_RX_ENB)) {
                        DELAY(100);
                        CSR_WRITE_1(sc, RE_COMMAND, CSR_READ_1(sc, RE_COMMAND) & ~(RE_CMD_TX_ENB | RE_CMD_RX_ENB));
                }

                for (i = 0; i < 3000; i++) {
                        DELAY(50);
                        if ((CSR_READ_1(sc, RE_MCU_CMD) & (RE_TXFIFO_EMPTY | RE_RXFIFO_EMPTY)) == (RE_TXFIFO_EMPTY | RE_RXFIFO_EMPTY))
                                break;
                }
                break;
        case MACFG_80:
        case MACFG_81:
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_3);
                DELAY(2000);

                if (CSR_READ_1(sc, RE_COMMAND) & (RE_CMD_TX_ENB | RE_CMD_RX_ENB)) {
                        DELAY(100);
                        CSR_WRITE_1(sc, RE_COMMAND, CSR_READ_1(sc, RE_COMMAND) & ~(RE_CMD_TX_ENB | RE_CMD_RX_ENB));
                }

                for (i = 0; i < 3000; i++) {
                        DELAY(50);
                        if ((CSR_READ_1(sc, RE_MCU_CMD) & (RE_TXFIFO_EMPTY | RE_RXFIFO_EMPTY)) == (RE_TXFIFO_EMPTY | RE_RXFIFO_EMPTY))
                                break;
                }
                break;
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_exit_oob_phy_8125(sc);
                break;
        }

        //Disable realwow function
        switch (sc->re_type) {
        case MACFG_50:
        case MACFG_51:
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xE5A90000);
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xF2100010);
                break;
        case MACFG_52:
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xE5A90000);
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xE4640000);
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xF2100010);
                break;
        case MACFG_56:
        case MACFG_57:
                CSR_WRITE_4(sc, RE_MCUACCESS, 0x605E0000);
                CSR_WRITE_4(sc, RE_MCUACCESS, (0xE05E << 16) | (CSR_READ_4(sc, RE_MCUACCESS) & 0xFFFE));
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xE9720000);
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xF2140010);
                break;
        case MACFG_60:
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xE05E00FF);
                CSR_WRITE_4(sc, RE_MCUACCESS, 0xE9720000);
                re_mac_ocp_write(sc, 0xE428, 0x0010);
                break;
        }

        if (sc->re_hw_supp_now_is_oob_ver >0)
                re_disable_now_is_oob(sc);

        switch(sc->re_type) {
        case MACFG_52:
                for (i = 0; i < 10; i++) {
                        DELAY(100);
                        if (CSR_READ_2(sc, 0xD2) & BIT_9)
                                break;
                }

                data16 = re_mac_ocp_read(sc, 0xD4DE) | BIT_15;
                re_mac_ocp_write(sc, 0xD4DE, data16);

                for (i = 0; i < 10; i++) {
                        DELAY(100);
                        if (CSR_READ_2(sc, 0xD2) & BIT_9)
                                break;
                }
                break;
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                data16 = re_mac_ocp_read(sc, 0xE8DE) & ~BIT_14;
                re_mac_ocp_write(sc, 0xE8DE, data16);
                for (i = 0; i < 10; i++) {
                        DELAY(100);
                        if (CSR_READ_2(sc, 0xD2) & BIT_9)
                                break;
                }

                data16 = re_mac_ocp_read(sc, 0xE8DE) | BIT_15;
                re_mac_ocp_write(sc, 0xE8DE, data16);

                for (i = 0; i < 10; i++) {
                        DELAY(100);
                        if (CSR_READ_2(sc, 0xD2) & BIT_9)
                                break;
                }
                break;
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                data16 = re_mac_ocp_read(sc, 0xE8DE) & ~BIT_14;
                re_mac_ocp_write(sc, 0xE8DE, data16);
                for (i = 0; i < 10; i++) {
                        DELAY(100);
                        if (CSR_READ_2(sc, 0xD2) & BIT_9)
                                break;
                }

                re_mac_ocp_write(sc, 0xC0AA, 0x07D0);
                re_mac_ocp_write(sc, 0xC0A6, 0x01B5);
                re_mac_ocp_write(sc, 0xC01E, 0x5555);

                for (i = 0; i < 10; i++) {
                        DELAY(100);
                        if (CSR_READ_2(sc, 0xD2) & BIT_9)
                                break;
                }
                break;
        }

        //wait ups resume (phy state 2)
        switch(sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                if (re_is_ups_resume(sc)) {
                        re_wait_phy_ups_resume(sc, 2);
                        re_clear_ups_resume_bit(sc);
                        re_clear_phy_ups_reg(sc);
                }
                break;
        };

        /*
        * Config MAC MCU
        */
        re_hw_mac_mcu_config(sc);
}

static void re_hw_init(struct re_softc *sc)
{
        /*
        * disable EDT.
        */
        switch(sc->re_type) {
        case MACFG_16:
        case MACFG_17:
        case MACFG_18:
        case MACFG_19:
        case MACFG_41:
                CSR_WRITE_1(sc, 0xF4, CSR_READ_1(sc, 0xF4) & ~(BIT_0|BIT_1));
                break;
        case MACFG_36:
        case MACFG_37:
        case MACFG_38:
        case MACFG_39:
        case MACFG_42:
        case MACFG_43:
        case MACFG_50:
        case MACFG_51:
        case MACFG_54:
        case MACFG_55:
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~(BIT_0|BIT_1|BIT_2));
                break;
        }

        re_enable_cfg9346_write(sc);

        if (s0_magic_packet == 0)
                re_disable_magic_packet(sc);
        else
                re_enable_magic_packet(sc);

        re_disable_cfg9346_write(sc);

        switch(sc->re_type) {
        case MACFG_5:
                if (CSR_READ_1(sc, RE_CFG2) & 1) {
                        CSR_WRITE_4(sc, 0x7C, 0x000FFFFF);
                } else {
                        CSR_WRITE_4(sc, 0x7C, 0x000FFF00);
                }
                break;
        case MACFG_6:
                if (CSR_READ_1(sc, RE_CFG2) & 1) {
                        CSR_WRITE_4(sc, 0x7C, 0x003FFFFF);
                } else {
                        CSR_WRITE_4(sc, 0x7C, 0x003FFF00);
                }
                break;
        }

        switch(sc->re_type) {
        case MACFG_33:
        case MACFG_36:
        case MACFG_37:
                CSR_WRITE_1(sc, 0xF3, CSR_READ_1(sc, 0xF3) | BIT_2);
                break;
        }

        switch(sc->re_type) {
        case MACFG_36:
        case MACFG_37:
        case MACFG_38:
        case MACFG_39:
        case MACFG_42:
        case MACFG_43:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
        case MACFG_54:
        case MACFG_55:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_enable_force_clkreq(sc, 0);
                re_enable_aspm_clkreq_lock(sc, 0);
                re_enable_cfg9346_write(sc);
                CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_5);
                re_disable_cfg9346_write(sc);
                break;
        }

        if (sc->re_if_flags & RL_FLAG_PCIE) {
                uint32_t Data32;
                //Set PCIE uncorrectable error status mask pcie 0x108
                Data32 = re_csi_read(sc, 0xF108);
                Data32 |= BIT_20;
                re_csi_write(sc, 0xF108, Data32);
        }
}

static void re_rar_set(struct re_softc *sc, u_int8_t *eaddr)
{
        re_enable_cfg9346_write(sc);

        CSR_WRITE_4(sc, RE_IDR0,
                    htole32(*(u_int32_t *)(&eaddr[0])));
        CSR_WRITE_2(sc, RE_IDR4,
                    htole16(*(u_int32_t *)(&eaddr[4])));

        switch (sc->re_type) {
        case MACFG_36:
        case MACFG_37:
        case MACFG_42:
        case MACFG_43:
        case MACFG_54:
        case MACFG_55:
                CSR_WRITE_4(sc, RE_SecMAC0,
                            htole32(*(u_int32_t *)(&eaddr[0])));
                CSR_WRITE_2(sc, RE_SecMAC4,
                            htole16(*(u_int16_t *)(&eaddr[4])));
                break;
        }

        switch (sc->re_type) {
        case MACFG_38:
        case MACFG_39:
                re_eri_write(sc, 0xF0, 4, *(u_int16_t *)(&eaddr[0])<<16, ERIAR_ExGMAC);
                re_eri_write(sc, 0xF4, 4, *(u_int32_t *)(&eaddr[2]), ERIAR_ExGMAC);
                break;
        }

        re_disable_cfg9346_write(sc);
}

static void re_get_hw_mac_address(struct re_softc *sc, u_int8_t *eaddr)
{
        device_t dev = sc->dev;
        u_int16_t re_eeid = 0;
        int i;

        for (i = 0; i < ETHER_ADDR_LEN; i++)
                eaddr[i] = CSR_READ_1(sc, RE_IDR0 + i);

        switch(sc->re_type) {
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                *(u_int32_t *)&eaddr[0] = re_eri_read(sc, 0xE0, 4, ERIAR_ExGMAC);
                *(u_int16_t *)&eaddr[4] = (u_int16_t)re_eri_read(sc, 0xE4, 4, ERIAR_ExGMAC);
                break;
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                *(u_int32_t *)&eaddr[0] = CSR_READ_4(sc, RE_BACKUP_ADDR0_8125);
                *(u_int16_t *)&eaddr[4] = CSR_READ_2(sc, RE_BACKUP_ADDR4_8125);
                break;
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
                break;
        default:
                re_read_eeprom(sc, (caddr_t)&re_eeid, RE_EE_ID, 1, 0);
                if (re_eeid == 0x8129)
                        re_read_eeprom(sc, (caddr_t)&eaddr[0], RE_EE_EADDR, 3, 0);
                break;
        }

        if (!is_valid_ether_addr(eaddr)) {
                device_printf(dev,"Invalid ether addr: %6D\n", eaddr, ":");
                ether_gen_addr(sc->re_ifp, (struct ether_addr *)eaddr);
                device_printf(dev,"Random ether addr: %6D\n", eaddr, ":");
                sc->random_mac = 1;
        }

        re_rar_set(sc, eaddr);
}

static int re_check_mac_version(struct re_softc *sc)
{
        device_t dev = sc->dev;
        int error = 0;

        switch(CSR_READ_4(sc, RE_TXCFG) & 0xFCF00000) {
        case RE_HWREV_8169S:
        case RE_HWREV_8110S:
                sc->re_type = MACFG_3;
                sc->max_jumbo_frame_size = Jumbo_Frame_7k;
                CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
                break;
        case RE_HWREV_8169_8110SB:
                sc->re_type = MACFG_4;
                sc->max_jumbo_frame_size = Jumbo_Frame_7k;
                CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
                break;
        case RE_HWREV_8169_8110SC:
                sc->re_type = MACFG_5;
                sc->max_jumbo_frame_size = Jumbo_Frame_7k;
                CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
                break;
        case 0x98000000:
                sc->re_type = MACFG_6;
                sc->max_jumbo_frame_size = Jumbo_Frame_7k;
                CSR_WRITE_4(sc, RE_RXCFG, 0xFF00);
                break;
        case RE_HWREV_8101E:
        case 0xB4000000:
                sc->re_type = MACFG_11;
                sc->max_jumbo_frame_size = ETHERMTU;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x34200000:
        case 0xB4200000:
                sc->re_type = MACFG_12;
                sc->max_jumbo_frame_size = ETHERMTU;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x34300000:
        case 0xB4300000:
                sc->re_type = MACFG_13;
                sc->max_jumbo_frame_size = ETHERMTU;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x34900000:
        case 0x24900000:
                sc->re_type = MACFG_14;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x34A00000:
        case 0x24A00000:
                sc->re_type = MACFG_15;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x34B00000:
        case 0x24B00000:
                sc->re_type = MACFG_16;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case RE_HWREV_8103E:
        case 0x24C00000:
                sc->re_type = MACFG_17;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x34D00000:
        case 0x24D00000:
                sc->re_type = MACFG_18;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x34E00000:
        case 0x24E00000:
                sc->re_type = MACFG_19;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x30000000:
                sc->re_type = MACFG_21;
                sc->max_jumbo_frame_size = Jumbo_Frame_4k;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case RE_HWREV_8168B_SPIN2:
                sc->re_type = MACFG_22;
                sc->max_jumbo_frame_size = Jumbo_Frame_4k;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x38500000:
        case 0xB8500000:
        case 0x38700000:
        case 0xB8700000:
                sc->re_type = MACFG_23;
                sc->max_jumbo_frame_size = Jumbo_Frame_4k;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x3C000000:
                sc->re_type = MACFG_24;
                sc->max_jumbo_frame_size = Jumbo_Frame_6k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
                break;
        case 0x3C200000:
                sc->re_type = MACFG_25;
                sc->max_jumbo_frame_size = Jumbo_Frame_6k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
                break;
        case 0x3C400000:
                sc->re_type = MACFG_26;
                sc->max_jumbo_frame_size = Jumbo_Frame_6k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
                break;
        case 0x3C900000:
                sc->re_type = MACFG_27;
                sc->max_jumbo_frame_size = Jumbo_Frame_6k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
                break;
        case 0x3CB00000:
                sc->re_type = MACFG_28;
                sc->max_jumbo_frame_size = Jumbo_Frame_6k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xC700);
                break;
        case 0x28100000:
                sc->re_type = MACFG_31;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x28200000:
                sc->re_type = MACFG_32;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x28300000:
                sc->re_type = MACFG_33;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x2C100000:
                sc->re_type = MACFG_36;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x2C200000:
                sc->re_type = MACFG_37;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x2C800000:
                sc->re_type = MACFG_38;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xBF00);
                break;
        case 0x2C900000:
                sc->re_type = MACFG_39;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xBF00);
                break;
        case 0x24000000:
                sc->re_type = MACFG_41;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x40900000:
                sc->re_type = MACFG_42;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x40A00000:
        case 0x40B00000:
        case 0x40C00000:
                sc->re_type = MACFG_43;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x48000000:
                sc->re_type = MACFG_50;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xBF00);
                break;
        case 0x48100000:
                sc->re_type = MACFG_51;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xBF00);
                break;
        case 0x48800000:
                sc->re_type = MACFG_52;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xBF00);
                break;
        case RE_HWREV_8402:
                sc->re_type = MACFG_53;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x44800000:
                sc->re_type = MACFG_54;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x44900000:
                sc->re_type = MACFG_55;
                sc->max_jumbo_frame_size = ETHERMTU;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0xE700);
                break;
        case 0x4C000000:
                sc->re_type = MACFG_56;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x4C100000:
                sc->re_type = MACFG_57;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x50800000:
                sc->re_type = MACFG_58;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x50900000:
                sc->re_type = MACFG_59;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x5C800000:
                sc->re_type = MACFG_60;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x50000000:
                sc->re_type = MACFG_61;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x50100000:
                sc->re_type = MACFG_62;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x50200000:
                sc->re_type = MACFG_67;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x28800000:
                sc->re_type = MACFG_63;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x28900000:
                sc->re_type = MACFG_64;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x28A00000:
                sc->re_type = MACFG_65;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x28B00000:
                sc->re_type = MACFG_66;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8700);
                break;
        case 0x54000000:
                sc->re_type = MACFG_68;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x54100000:
                sc->re_type = MACFG_69;
                if ((re_mac_ocp_read(sc, 0xD006) & 0xFF00) == 0x0100)
                        sc->re_type = MACFG_74;
                else if ((re_mac_ocp_read(sc, 0xD006) & 0xFF00) == 0x0300)
                        sc->re_type = MACFG_75;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x6C000000:
                sc->re_type = MACFG_76;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_8168G_PLUS |
                                   RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x54900000:
                sc->re_type = MACFG_70;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x54A00000:
                sc->re_type = MACFG_71;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x54B00000:
                sc->re_type = MACFG_72;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x54C00000:
                sc->re_type = MACFG_73;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V2;
                CSR_WRITE_4(sc, RE_RXCFG, 0x8F00);
                break;
        case 0x60800000:
                sc->re_type = MACFG_80;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00400);
                break;
        case 0x60900000:
                sc->re_type = MACFG_81;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00400);
                break;
        case RE_HWREV_8125B_REV_A:
                sc->re_type = MACFG_82;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00C00);
                break;
        case RE_HWREV_8125B_REV_B:
                sc->re_type = MACFG_83;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00C00);
                break;
        case 0x68000000:
                sc->re_type = MACFG_84;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00C00);
                break;
        case 0x68100000:
                sc->re_type = MACFG_85;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00C00);
                break;
        case 0x68800000:
                sc->re_type = MACFG_86;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00C00);
                break;
        case 0x68900000:
                sc->re_type = MACFG_87;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00C00);
                break;
        case 0x64800000:
                sc->re_type = MACFG_90;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00F00);
                break;
        case 0x64900000:
                sc->re_type = MACFG_91;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00D00);
                break;
        case 0x64A00000:
                sc->re_type = MACFG_92;
                sc->max_jumbo_frame_size = Jumbo_Frame_9k;
                sc->re_if_flags |= RL_FLAG_DESCV2 | RL_FLAG_PHYWAKE_PM |
                                   RL_FLAG_8168G_PLUS | RL_FLAG_MAGIC_PACKET_V3;
                CSR_WRITE_4(sc, RE_RXCFG, 0x40C00D00);
                break;
        default:
                device_printf(dev,"unknown device\n");
                sc->re_type = MACFG_FF;
                error = ENXIO;
                break;
        }

        switch(sc->re_device_id) {
        case RT_DEVICEID_8169:
        case RT_DEVICEID_8169SC:
        case RT_DEVICEID_8168:
        case RT_DEVICEID_8161:
        case RT_DEVICEID_8162:
        case RT_DEVICEID_8125:
        case RT_DEVICEID_3000:
        case RT_DEVICEID_8126:
                //do nothing
                break;
        default:
                sc->max_jumbo_frame_size = ETHERMTU;
                break;
        }

        return error;
}

static bool
re_is_allow_access_dash_ocp(struct re_softc *sc)
{
        bool allow_access = false;
        u_int16_t mac_ocp_data;

        if (!HW_SUPPORT_OCP_CHANNEL(sc))
                goto exit;

        switch (sc->re_type) {
        case MACFG_80:
        case MACFG_81:
                mac_ocp_data = re_mac_ocp_read(sc, 0xd460);
                if (mac_ocp_data == 0xffff || !(mac_ocp_data & BIT_0))
                        goto exit;
                break;
        case MACFG_84:
        case MACFG_85:
                mac_ocp_data = re_mac_ocp_read(sc, 0xd4c0);
                if (mac_ocp_data == 0xffff || (mac_ocp_data & BIT_3))
                        goto exit;
                break;
        default:
                break;
        }

        allow_access = true;

exit:
        return allow_access;
}

static void re_init_software_variable(struct re_softc *sc)
{
        switch(sc->re_device_id) {
        case RT_DEVICEID_8168:
        case RT_DEVICEID_8161:
        case RT_DEVICEID_8162:
        case RT_DEVICEID_8136:
        case RT_DEVICEID_8125:
        case RT_DEVICEID_3000:
        case RT_DEVICEID_8126:
                sc->re_if_flags |= RL_FLAG_PCIE;
                break;
        }

        sc->re_rx_mbuf_sz = sc->max_jumbo_frame_size + ETHER_VLAN_ENCAP_LEN + ETHER_HDR_LEN + ETHER_CRC_LEN + RE_ETHER_ALIGN + 1;

        if (sc->re_rx_mbuf_sz > max_rx_mbuf_sz) {
                sc->max_jumbo_frame_size -= (sc->re_rx_mbuf_sz - max_rx_mbuf_sz);
                sc->re_rx_mbuf_sz = max_rx_mbuf_sz;
        }

        switch(sc->re_type) {
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
                sc->HwSuppDashVer = 1;
                break;
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
                sc->HwSuppDashVer = 2;
                break;
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
                sc->HwSuppDashVer = 3;
                break;
        case MACFG_80:
        case MACFG_81: {
                u_int8_t tmpUchar;
                tmpUchar = (u_int8_t)re_mac_ocp_read(sc, 0xD006);
                if (tmpUchar == 0x02 || tmpUchar == 0x04)
                        sc->HwSuppDashVer = 2;
        }
        break;
        case MACFG_84:
        case MACFG_85:
                sc->HwSuppDashVer = 4;
                break;
        default:
                sc->HwSuppDashVer = 0;
                break;
        }

        switch(sc->re_type) {
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
                sc->HwSuppOcpChannelVer = 1;
                break;
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_84:
        case MACFG_85:
                sc->HwSuppOcpChannelVer = 2;
                break;
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
                sc->HwSuppOcpChannelVer = 3;
                break;
        case MACFG_80:
        case MACFG_81:
                if (sc->HwSuppDashVer > 0)
                        sc->HwSuppOcpChannelVer = 2;
                break;
        default:
                sc->HwSuppOcpChannelVer = 0;
                break;
        }

        switch(sc->re_type) {
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
                sc->HwPkgDet = re_mac_ocp_read(sc, 0xDC00);
                sc->HwPkgDet = (sc->HwPkgDet >> 3) & 0x0F;
                break;
        }

        switch(sc->re_type) {
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
                if (sc->HwPkgDet == 0x06) {
                        u_int8_t tmpUchar = re_eri_read(sc, 0xE6, 1, ERIAR_ExGMAC);
                        if (tmpUchar == 0x02)
                                sc->hw_hw_supp_serdes_phy_ver = 1;
                        else if (tmpUchar == 0x00)
                                sc->hw_hw_supp_serdes_phy_ver = 2;
                }
                break;
        }

        if (HW_SUPP_SERDES_PHY(sc))
                eee_enable = 0;

        if (HW_DASH_SUPPORT_DASH(sc)) {
                sc->AllowAccessDashOcp = re_is_allow_access_dash_ocp(sc);
                sc->re_dash = re_check_dash(sc);
        }

        if (sc->re_dash) {
#if defined(__amd64__) || defined(__i386__)
                if (HW_DASH_SUPPORT_TYPE_3(sc)) {
                        u_int64_t CmacMemPhysAddress;
                        bus_space_handle_t cmac_ioaddr;

                        CmacMemPhysAddress = re_csi_other_fun_read(sc, 0, 0xf018);
                        if (!(CmacMemPhysAddress & BIT_0)) {
                                if (CmacMemPhysAddress & BIT_2)
                                        CmacMemPhysAddress |=  (u_int64_t)re_csi_other_fun_read(sc, 0, 0xf01c) << 32;

                                CmacMemPhysAddress &=  0xFFFFFFF0;
                                /* ioremap MMIO region */
                                sc->re_mapped_cmac_tag = X86_BUS_SPACE_MEM;
                                if (bus_space_map(sc->re_mapped_cmac_tag, CmacMemPhysAddress, RE_REGS_SIZE, 0,
                                                  &cmac_ioaddr))
                                        sc->re_dash = 0;
                                else
                                        sc->re_mapped_cmac_handle = cmac_ioaddr;
                        }
                }
#else
                sc->re_dash = 0;
#endif
        }

        switch(sc->re_type) {
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
                sc->re_cmac_handle = sc->re_bhandle;
                sc->re_cmac_tag = sc->re_btag;
                break;
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
                sc->re_cmac_handle = sc->re_mapped_cmac_handle;
                sc->re_cmac_tag = sc->re_mapped_cmac_tag;
                break;
        }

        switch(sc->re_type) {
        case MACFG_14:
        case MACFG_15:
        case MACFG_16:
        case MACFG_17:
        case MACFG_18:
        case MACFG_19:
        case MACFG_31:
        case MACFG_32:
        case MACFG_33:
        case MACFG_41:
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
                sc->re_efuse_ver = EFUSE_SUPPORT_V1;
                break;
        case MACFG_36:
        case MACFG_37:
        case MACFG_42:
        case MACFG_43:
        case MACFG_54:
        case MACFG_55:
                sc->re_efuse_ver = EFUSE_SUPPORT_V2;
                break;
        case MACFG_38:
        case MACFG_39:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                sc->re_efuse_ver = EFUSE_SUPPORT_V3;
                break;
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                sc->re_efuse_ver = EFUSE_SUPPORT_V4;
                break;
        default:
                sc->re_efuse_ver = EFUSE_NOT_SUPPORT;
                break;
        }

        switch(sc->re_type) {
        case MACFG_69:
        case MACFG_76: {
                u_int16_t ioffset_p3, ioffset_p2, ioffset_p1, ioffset_p0;
                u_int16_t TmpUshort;

                re_mac_ocp_write(sc, 0xDD02, 0x807D);

                TmpUshort = re_mac_ocp_read(sc, 0xDD02);
                ioffset_p3 = ((TmpUshort & BIT_7) >>7);
                ioffset_p3 <<= 3;
                TmpUshort = re_mac_ocp_read(sc, 0xDD00);

                ioffset_p3 |= ((TmpUshort & (BIT_15 | BIT_14 | BIT_13))>>13);

                ioffset_p2 = ((TmpUshort & (BIT_12|BIT_11|BIT_10|BIT_9))>>9);
                ioffset_p1 = ((TmpUshort & (BIT_8|BIT_7|BIT_6|BIT_5))>>5);

                ioffset_p0 = ((TmpUshort & BIT_4) >>4);
                ioffset_p0 <<= 3;
                ioffset_p0 |= (TmpUshort & (BIT_2| BIT_1 | BIT_0));

                if ((ioffset_p3 == 0x0F) && (ioffset_p2 == 0x0F) && (ioffset_p1 == 0x0F) && (ioffset_p0 == 0x0F)) {
                        sc->RequireAdcBiasPatch = FALSE;
                } else {
                        sc->RequireAdcBiasPatch = TRUE;
                        sc->AdcBiasPatchIoffset = (ioffset_p3<<12)|(ioffset_p2<<8)|(ioffset_p1<<4)|(ioffset_p0);
                }
        }
        break;
        }

        switch(sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_75:
        case MACFG_76: {
                u_int16_t rg_saw_cnt;

                re_mdio_write(sc, 0x1F, 0x0C42);
                rg_saw_cnt = re_mdio_read(sc, 0x13);
                rg_saw_cnt &= ~(BIT_15|BIT_14);
                re_mdio_write(sc, 0x1F, 0x0000);

                if (rg_saw_cnt > 0) {
                        sc->SwrCnt1msIni = 16000000/rg_saw_cnt;
                        sc->SwrCnt1msIni &= 0x0FFF;

                        sc->RequireAdjustUpsTxLinkPulseTiming = TRUE;
                }
        }
        break;
        }

#ifdef ENABLE_FIBER_SUPPORT
        re_check_hw_fiber_mode_support(sc);
#endif //ENABLE_FIBER_SUPPORT

        switch (sc->re_type) {
        case MACFG_74:
        case MACFG_75:
                sc->RequiredSecLanDonglePatch = FALSE;
                break;
        }

        switch(sc->re_type) {
        case MACFG_31:
        case MACFG_32:
        case MACFG_33:
        case MACFG_36:
        case MACFG_37:
        case MACFG_38:
        case MACFG_39:
        case MACFG_42:
        case MACFG_43:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
        case MACFG_54:
        case MACFG_55:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                sc->re_hw_enable_msi_msix = TRUE;
                break;
        }

        switch(sc->re_type) {
        case MACFG_3:
        case MACFG_4:
        case MACFG_5:
        case MACFG_6:
        case MACFG_11:
        case MACFG_12:
        case MACFG_13:
        case MACFG_21:
        case MACFG_22:
        case MACFG_23:
        case MACFG_24:
        case MACFG_25:
        case MACFG_26:
        case MACFG_27:
        case MACFG_28:
        case MACFG_41:
        case MACFG_42:
        case MACFG_43:
        case MACFG_54:
        case MACFG_55:
                sc->re_coalesce_tx_pkt = TRUE;
                break;
        }

        switch(sc->re_type) {
        case MACFG_36:
        case MACFG_37:
        case MACFG_38:
        case MACFG_39:
        case MACFG_42:
        case MACFG_43:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
        case MACFG_54:
        case MACFG_55:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                sc->re_hw_supp_now_is_oob_ver = 1;
                break;
        }

        switch (sc->re_type) {
        case MACFG_36:
        case MACFG_37:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168E;
                break;
        case MACFG_38:
        case MACFG_39:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168EVL;
                break;
        case MACFG_50:
        case MACFG_51:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168F;
                break;
        case MACFG_52:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8411;
                break;
        case MACFG_56:
        case MACFG_57:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168G;
                break;
        case MACFG_58:
        case MACFG_59:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168GU;
                break;
        case MACFG_60:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8411B;
                break;
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168EP;
                break;
        case MACFG_68:
        case MACFG_69:
        case MACFG_76:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168H;
                break;
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168FP;
                break;
        case MACFG_74:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168H_6838;
                break;
        case MACFG_75:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8168H_6878B;
                break;
        case MACFG_80:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125A_REV_A;
                break;
        case MACFG_81:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125A_REV_B;
                break;
        case MACFG_82:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125B_REV_A;
                break;
        case MACFG_83:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125B_REV_B;
                break;
        case MACFG_84:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125BP_REV_A;
                break;
        case MACFG_85:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125BP_REV_B;
                break;
        case MACFG_86:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125D_REV_A;
                break;
        case MACFG_87:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8125D_REV_B;
                break;
        case MACFG_90:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8126A_REV_A;
                break;
        case MACFG_91:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8126A_REV_B;
                break;
        case MACFG_92:
                sc->re_sw_ram_code_ver = NIC_RAMCODE_VERSION_8126A_REV_C;
                break;
        }

        switch (sc->re_type) {
        case MACFG_81:
                if ((re_mac_ocp_read(sc, 0xD442) & BIT_5) &&
                    (re_real_ocp_phy_read(sc, 0xD068) & BIT_1)
                   ) {
                        sc->RequirePhyMdiSwapPatch = TRUE;
                }
                break;
        }

        switch (sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                sc->HwSuppExtendTallyCounterVer = 1;
                break;
        }

        switch (sc->re_type) {
        case MACFG_38:
        case MACFG_39:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                sc->HwSuppMacMcuVer = 1;
                break;
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                sc->HwSuppMacMcuVer = 2;
                break;
        }

        switch (sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                sc->MacMcuPageSize = RTL8125_MAC_MCU_PAGE_SIZE;
                break;
        }

        switch (sc->re_type) {
        case MACFG_84:
        case MACFG_85:
                sc->RequiredPfmPatch = TRUE;
                break;
        }

        sc->re_8169_MacVersion = (CSR_READ_4(sc, RE_TXCFG)&0x7c800000)>>25;		/* Get bit 26~30 	*/
        sc->re_8169_MacVersion |= ((CSR_READ_4(sc, RE_TXCFG)&0x00800000)!=0 ? 1:0);	/* Get bit 23 		*/
        DBGPRINT1(sc->re_unit,"8169 Mac Version %d",sc->re_8169_MacVersion);

        /* Rtl8169s single chip detected */
        if (sc->re_type == MACFG_3) {
                RE_LOCK(sc);
                sc->re_8169_PhyVersion=(re_mdio_read(sc, 0x03)&0x000f);
                DBGPRINT1(sc->re_unit,"8169 Phy Version %d",sc->re_8169_PhyVersion);
                RE_UNLOCK(sc);
        }

        sc->link_state = LINK_STATE_UNKNOWN;

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(sc))
                re_set_fiber_mode_software_variable(sc);
#endif //ENABLE_FIBER_SUPPORT
}

static void re_enable_ocp_phy_power_saving(struct re_softc *sc)
{
        u_int16_t val;

        if (sc->re_type == MACFG_59 || sc->re_type == MACFG_60 ||
            sc->re_type == MACFG_62 || sc->re_type == MACFG_67 ||
            sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
            sc->re_type == MACFG_70 || sc->re_type == MACFG_71 ||
            sc->re_type == MACFG_72 || sc->re_type == MACFG_73 ||
            sc->re_type == MACFG_74 || sc->re_type == MACFG_75 ||
            sc->re_type == MACFG_76) {
                val = re_ocp_phy_read(sc, 0x0C41, 0x13);
                if (val != 0x0050) {
                        re_set_phy_mcu_patch_request(sc);
                        re_ocp_phy_write(sc, 0x0C41, 0x13, 0x0000);
                        re_ocp_phy_write(sc, 0x0C41, 0x13, 0x0050);
                        re_clear_phy_mcu_patch_request(sc);
                }
        } else if (sc->re_type == MACFG_80 || sc->re_type == MACFG_81) {
                val = re_real_ocp_phy_read(sc, 0xC416);
                if (val != 0x0050) {
                        re_set_phy_mcu_patch_request(sc);
                        re_real_ocp_phy_write(sc, 0xC416, 0x0000);
                        re_real_ocp_phy_write(sc, 0xC416, 0x0050);
                        re_clear_phy_mcu_patch_request(sc);
                }
        }
}

static void re_disable_ocp_phy_power_saving(struct re_softc *sc)
{
        u_int16_t val;

        if (sc->re_type == MACFG_59 || sc->re_type == MACFG_60 ||
            sc->re_type == MACFG_62 || sc->re_type == MACFG_67 ||
            sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
            sc->re_type == MACFG_70 || sc->re_type == MACFG_71 ||
            sc->re_type == MACFG_72 || sc->re_type == MACFG_73 ||
            sc->re_type == MACFG_74 || sc->re_type == MACFG_75 ||
            sc->re_type == MACFG_76) {
                val = re_ocp_phy_read(sc, 0x0C41, 0x13);
                if (val != 0x0500) {
                        re_set_phy_mcu_patch_request(sc);
                        re_ocp_phy_write(sc, 0x0C41, 0x13, 0x0000);
                        re_ocp_phy_write(sc, 0x0C41, 0x13, 0x0500);
                        re_clear_phy_mcu_patch_request(sc);
                }
        } else if (sc->re_type == MACFG_80 || sc->re_type == MACFG_81) {
                val = re_real_ocp_phy_read(sc, 0xC416);
                if (val != 0x0500) {
                        re_set_phy_mcu_patch_request(sc);
                        re_real_ocp_phy_write(sc, 0xC416, 0x0000);
                        re_real_ocp_phy_write(sc, 0xC416, 0x0500);
                        re_clear_phy_mcu_patch_request(sc);
                }
        }
}

static void re_hw_d3_para(struct re_softc *sc)
{
        switch (sc->re_type) {
        case MACFG_59:
        case MACFG_60:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
        case MACFG_80:
        case MACFG_81:
                re_disable_ocp_phy_power_saving(sc);
                break;
        }
}

static void
re_add_sysctls(struct re_softc *sc)
{
        struct sysctl_ctx_list  *ctx;
        struct sysctl_oid_list  *children;

        ctx = device_get_sysctl_ctx(sc->dev);
        children = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

#ifndef CTLFLAG_NEEDGIANT
#define CTLFLAG_NEEDGIANT 0
#endif

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "driver_var",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_driver_variable, "I", "Driver Variables Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "stats",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_stats, "I", "Statistics Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "registers",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_registers, "I", "MAC IO Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "registers2",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_registers2, "I", "MAC IO 0x0000 to 0x4000 Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "registers3",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_registers3, "I", "MAC IO 0x4000 to 0x8000 Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "registers4",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_registers4, "I", "MAC IO 0x8000 to 0xC000 Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "registers5",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_registers5, "I", "MAC IO 0xC000 to 0x10000 Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "eth_phy",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_eth_phy, "I", "Ethernet PHY Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "rx_desc",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_dump_rx_desc, "I", "RX Descriptor Information");

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "tx_desc",
                        CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                        re_sysctl_dump_tx_desc, "I", "TX Descriptor Information");

        if ((sc->re_if_flags & RL_FLAG_PCIE) != 0) {
                SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "pcie_phy",
                                CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                                re_sysctl_pcie_phy, "I", "PCIE PHY Information");

                SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "ext_regs",
                                CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                                re_sysctl_extended_registers, "I", "Extended Registers Information");

                SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "pci_regs",
                                CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                                re_sysctl_pci_registers, "I", "PCI Configuration Information");

                SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "msix_tbl",
                                CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_NEEDGIANT, sc, 0,
                                re_sysctl_msix_tbl, "I", "MSIX Table Information");
        }
}

static int
re_sysctl_driver_variable(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                printf("%s Driver Variables:\n", device_get_nameunit(sc->dev));

                printf("driver version\t%s\n", RE_VERSION);
                printf("if_drv_flags\t0x%08x\n", sc->re_ifp->if_drv_flags);
                printf("re_type\t%d\n", sc->re_type);
                printf("re_res_id\t%d\n", sc->re_res_id);
                printf("re_res_type\t%d\n", sc->re_res_type);
                printf("re_8169_MacVersion\t%d\n", sc->re_8169_MacVersion);
                printf("re_8169_PhyVersion\t%d\n", sc->re_8169_PhyVersion);
                printf("tx buffer numer\t%d\n", RE_TX_BUF_NUM);
                printf("rx buffer numer\t%d\n", RE_RX_BUF_NUM);
                printf("rx_cur_index\t%d\n", sc->re_desc.rx_cur_index);
                printf("tx_cur_index\t%d\n", sc->re_desc.tx_cur_index);
                printf("tx_last_index\t%d\n", sc->re_desc.tx_last_index);
                printf("rx_fifo_overflow\t%d\n", sc->rx_fifo_overflow);
                printf("driver_detach\t%d\n", sc->driver_detach);
                printf("interface name\tre%d\n", sc->re_unit);
                printf("re_revid\t0x%02x\n", sc->re_revid);
                printf("re_vendor_id\t0x%04x\n", sc->re_vendor_id);
                printf("re_device_id\t0x%04x\n", sc->re_device_id);
                printf("re_subvendor_id\t0x%04x\n", sc->re_subvendor_id);
                printf("re_subdevice_id\t0x%04x\n", sc->re_subdevice_id);
                printf("max_jumbo_frame_size\t%d\n", sc->max_jumbo_frame_size);
                printf("re_rx_mbuf_sz\t%d\n", sc->re_rx_mbuf_sz);
                printf("re_rx_desc_buf_sz\t%d\n", sc->re_rx_desc_buf_sz);
                printf("re_if_flags\t0x%08x\n", sc->re_if_flags);
                printf("re_tx_cstag\t%d\n", sc->re_tx_cstag);
                printf("re_rx_cstag\t%d\n", sc->re_rx_cstag);
                printf("RequireAdcBiasPatch\t%d\n", sc->RequireAdcBiasPatch);
                printf("RequireAdjustUpsTxLinkPulseTiming\t%d\n", sc->RequireAdjustUpsTxLinkPulseTiming);
                printf("RequiredSecLanDonglePatch\t%d\n", sc->RequiredSecLanDonglePatch);
                printf("RequiredPfmPatch\t%d\n", sc->RequiredPfmPatch);
                printf("RequirePhyMdiSwapPatch\t%d\n", sc->RequirePhyMdiSwapPatch);
                printf("re_efuse_ver\t%d\n", sc->re_efuse_ver);
                printf("re_sw_ram_code_ver\t0x%x\n", sc->re_sw_ram_code_ver);
                printf("re_hw_ram_code_ver\t0x%x\n", sc->re_hw_ram_code_ver);
                printf("cur_page\t0x%x\n", sc->cur_page);
                printf("phy_reg_anlpar\t0x%x\n", sc->phy_reg_anlpar);
                printf("re_hw_enable_msi_msix\t%d\n", sc->re_hw_enable_msi_msix);
                printf("re_coalesce_tx_pkt\t%d\n", sc->re_coalesce_tx_pkt);
                printf("link_state\t%s\n", sc->link_state==2?"up":(sc->link_state==1?"down":"unknown"));
                printf("prohibit_access_reg\t%d\n", sc->prohibit_access_reg);
                printf("re_hw_supp_now_is_oob_ver\t%d\n", sc->re_hw_supp_now_is_oob_ver);
                printf("hw_hw_supp_serdes_phy_ver\t%d\n", sc->hw_hw_supp_serdes_phy_ver);
                printf("HwSuppDashVer\t%d\n", sc->HwSuppDashVer);
                printf("re_dash\t%d\n", sc->re_dash);
                printf("re_dash_fw_ver\t0x%08x\n", sc->re_dash_fw_ver);
                printf("HwPkgDet\t%d\n", sc->HwPkgDet);
                printf("HwFiberModeVer\t%d\n", sc->HwFiberModeVer);
                printf("HwFiberStat\t%d\n", sc->HwFiberStat);
                printf("HwSuppExtendTallyCounterVer\t%d\n", sc->HwSuppExtendTallyCounterVer);
                printf("HwSuppMacMcuVer\t%d\n", sc->HwSuppMacMcuVer);
                printf("MacMcuPageSize\t%d\n", sc->MacMcuPageSize);
                printf("rx_desc_tag maxsize\t%zd\n", sc->re_desc.rx_desc_tag->common.maxsize);
                printf("tx_desc_tag maxsize\t%zd\n", sc->re_desc.tx_desc_tag->common.maxsize);
                printf("re_tally maxsize\t%zd\n", sc->re_tally.re_stag->common.maxsize);
                printf("random_mac\t%d\n", sc->random_mac);
                printf("org_mac_addr\t%6D\n", sc->org_mac_addr, ":");
#if OS_VER < VERSION(6,0)
                printf("dev_addr\t%6D\n", (char *)&sc->arpcom.ac_enaddr, ":");
#elif OS_VER < VERSION(7,0)
                printf("dev_addr\t%6D\n", IFP2ENADDR(sc->re_ifp), ":");
#else
                printf("dev_addr\t%6D\n", IF_LLADDR(sc->re_ifp), ":");
#endif
                printf("msi_disable\t%d\n", msi_disable);
                printf("msix_disable\t%d\n", msix_disable);
                printf("eee_enable\t%d\n", eee_enable);
                printf("prefer_iomap\t%d\n", prefer_iomap);
                printf("phy_power_saving\t%d\n", phy_power_saving);
                printf("phy_mdix_mode\t%d\n", phy_mdix_mode);
                printf("s5wol\t%d\n", s5wol);
                printf("s0_magic_packet\t%d\n", s0_magic_packet);
                printf("config_soc_lan\t%d\n", config_soc_lan);
                printf("interrupt_mitigation\t%d\n", interrupt_mitigation);
                printf("re_lro_entry_count\t%d\n", re_lro_entry_count);
                printf("re_lro_mbufq_depth\t%d\n", re_lro_mbufq_depth);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_stats(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        struct re_stats         *stats;
        int                     error, i, result;
        bool                    extend_stats;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);
                extend_stats = false;
                if (sc->HwSuppExtendTallyCounterVer > 0)
                        extend_stats = true;
                if ((sc->re_ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
                        RE_UNLOCK(sc);
                        goto done;
                }

                if (extend_stats)
                        re_set_mac_ocp_bit(sc, 0xEA84, (BIT_1 | BIT_0));

                bus_dmamap_sync(sc->re_tally.re_stag,
                                sc->re_tally.re_smap, BUS_DMASYNC_PREREAD);
                CSR_WRITE_4(sc, RE_DUMPSTATS_HI,
                            RL_ADDR_HI(sc->re_tally.re_stats_addr));
                CSR_WRITE_4(sc, RE_DUMPSTATS_LO,
                            RL_ADDR_LO(sc->re_tally.re_stats_addr));
                CSR_WRITE_4(sc, RE_DUMPSTATS_LO,
                            RL_ADDR_LO(sc->re_tally.re_stats_addr |
                                       RE_DUMPSTATS_START));
                for (i = RE_TIMEOUT; i > 0; i--) {
                        if ((CSR_READ_4(sc, RE_DUMPSTATS_LO) &
                             RE_DUMPSTATS_START) == 0)
                                break;
                        DELAY(1000);
                }
                bus_dmamap_sync(sc->re_tally.re_stag,
                                sc->re_tally.re_smap, BUS_DMASYNC_POSTREAD);

                if (extend_stats)
                        re_clear_mac_ocp_bit(sc, 0xEA84, (BIT_1 | BIT_0));

                RE_UNLOCK(sc);
                if (i == 0) {
                        device_printf(sc->dev,
                                      "DUMP statistics request timed out\n");
                        return (ETIMEDOUT);
                }
done:
                stats = sc->re_tally.re_stats;
                printf("%s statistics:\n", device_get_nameunit(sc->dev));
                printf("Tx frames : %ju\n",
                       (uintmax_t)le64toh(stats->re_tx_pkts));
                printf("Rx frames : %ju\n",
                       (uintmax_t)le64toh(stats->re_rx_pkts));
                printf("Tx errors : %ju\n",
                       (uintmax_t)le64toh(stats->re_tx_errs));
                printf("Rx errors : %u\n",
                       le32toh(stats->re_rx_errs));
                printf("Rx missed frames : %u\n",
                       (uint32_t)le16toh(stats->re_missed_pkts));
                printf("Rx frame alignment errs : %u\n",
                       (uint32_t)le16toh(stats->re_rx_framealign_errs));
                printf("Tx single collisions : %u\n",
                       le32toh(stats->re_tx_onecoll));
                printf("Tx multiple collisions : %u\n",
                       le32toh(stats->re_tx_multicolls));
                printf("Rx unicast frames : %ju\n",
                       (uintmax_t)le64toh(stats->re_rx_ucasts));
                printf("Rx broadcast frames : %ju\n",
                       (uintmax_t)le64toh(stats->re_rx_bcasts));
                printf("Rx multicast frames : %u\n",
                       le32toh(stats->re_rx_mcasts));
                printf("Tx aborts : %u\n",
                       (uint32_t)le16toh(stats->re_tx_aborts));
                printf("Tx underruns : %u\n",
                       (uint32_t)le16toh(stats->re_rx_underruns));

                if (extend_stats) {
                        printf("%s extend statistics:\n", device_get_nameunit(sc->dev));
                        printf("Tx octets : %ju\n",
                               (uintmax_t)le64toh(stats->re_tx_octets));
                        printf("Rx octets : %ju\n",
                               (uintmax_t)le64toh(stats->re_rx_octets));
                        printf("Rx multicast64 : %ju\n",
                               (uintmax_t)le64toh(stats->re_rx_multicast64));
                        printf("Rx unicast64 : %ju\n",
                               (uintmax_t)le64toh(stats->re_tx_unicast64));
                        printf("Tx broadcast64 : %ju\n",
                               (uintmax_t)le64toh(stats->re_tx_broadcast64));
                        printf("Tx multicast64 : %ju\n",
                               (uintmax_t)le64toh(stats->re_tx_multicast64));
                        printf("Tx pause on frames : %u\n",
                               (uint32_t)le32toh(stats->re_tx_pause_on));
                        printf("TTx pause off frames : %u\n",
                               (uint32_t)le32toh(stats->re_tx_pause_off));
                        printf("Tx pause all frames : %u\n",
                               (uint32_t)le32toh(stats->re_tx_pause_all));
                        printf("Tx deferred frames : %u\n",
                               (uint32_t)le32toh(stats->re_tx_deferred));
                        printf("Tx late collisions : %u\n",
                               (uint32_t)le32toh(stats->re_tx_late_collision));
                        printf("Tx all collisions : %u\n",
                               (uint32_t)le32toh(stats->re_tx_all_collision));
                        printf("Tx aborts32 : %u\n",
                               (uint32_t)le32toh(stats->re_tx_aborted32));
                        printf("Rx alignment errs32 : %u\n",
                               (uint32_t)le32toh(stats->re_align_errors32));
                        printf("Rx frame too long : %u\n",
                               (uint32_t)le32toh(stats->re_rx_frame_too_long));
                        printf("Rx runt : %u\n",
                               (uint32_t)le32toh(stats->re_rx_runt));
                        printf("Rx pause on frames : %u\n",
                               (uint32_t)le32toh(stats->re_rx_pause_on));
                        printf("Rx pause off frames : %u\n",
                               (uint32_t)le32toh(stats->re_rx_pause_off));
                        printf("Rx pause all frames : %u\n",
                               (uint32_t)le32toh(stats->re_rx_pause_all));
                        printf("Rx unknown opcode : %u\n",
                               (uint32_t)le32toh(stats->re_rx_unknown_opcode));
                        printf("Rx mac error : %u\n",
                               (uint32_t)le32toh(stats->re_rx_mac_error));
                        printf("Tx underruns32 : %u\n",
                               (uint32_t)le32toh(stats->re_tx_underrun32));
                        printf("Rx mac missed : %u\n",
                               (uint32_t)le32toh(stats->re_rx_mac_missed));
                        printf("Rx tcam drops : %u\n",
                               (uint32_t)le32toh(stats->re_rx_tcam_dropped));
                        printf("Tx desc unavailable : %u\n",
                               (uint32_t)le32toh(stats->re_tdu));
                        printf("Rx desc unavailable : %u\n",
                               (uint32_t)le32toh(stats->re_rdu));
                }
        }

        return (error);
}

static void
re_printf_macio(struct re_softc *sc,
                u_int32_t start,
                u_int32_t end)
{
        int i, n;

        printf("\n%s mac io start:0x%05x end:0x%05x:\n",
               device_get_nameunit(sc->dev),
               start, end);

        for (n=start; n<end;) {
                printf("\n0x%02x:\t", n);

                for (i=0; i<16 && n<end; i++, n++)
                        printf("%02x ", CSR_READ_1(sc, n));
        }

        return;
}

static int
re_sysctl_registers(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                max = min(256, rman_get_size(sc->re_res));
                re_printf_macio(sc, 0, max);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_registers2(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                max = min(0x4000, rman_get_size(sc->re_res));
                re_printf_macio(sc, 0, max);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_registers3(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                max = min(0x8000, rman_get_size(sc->re_res));
                re_printf_macio(sc, 0x4000, max);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_registers4(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                max = min(0xC000, rman_get_size(sc->re_res));
                re_printf_macio(sc, 0x8000, max);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_registers5(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                max = min(0x10000, rman_get_size(sc->re_res));
                re_printf_macio(sc, 0xC000, max);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_eth_phy(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, i, n, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                printf("%s ethernet phy:\n", device_get_nameunit(sc->dev));

                max = 16;
                re_mdio_write(sc, 0x1F, 0x0000);
                for (n=0; n<max;) {
                        printf("\n0x%02x:\t", n);

                        for (i = 0; i < 8 && n < max; i++, n++)
                                printf("%04x ", re_mdio_read(sc, n));
                }

                RE_UNLOCK(sc);
        }

        return (error);
}

static void
re_dump_desc(void *desc_base, uint32_t alloc_size)
{
        uint32_t *pdword;
        int i;

        if (desc_base == NULL ||
            alloc_size == 0)
                return;

        pdword = (uint32_t*)desc_base;
        for (i=0; i<(alloc_size/4); i++) {
                if (!(i % 4))
                        printf("\n%04x ", i);
                printf("%08x ", pdword[i]);
        }

        printf("\n");
        return;
}

static int
re_sysctl_dump_rx_desc(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        bus_size_t              rx_list_size;
        int                     error, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                printf("%s rx desc:%d\n", device_get_nameunit(sc->dev),
                       RE_RX_BUF_NUM);

                rx_list_size = sc->re_desc.rx_desc_tag->common.maxsize;
                re_dump_desc((void*)sc->re_desc.rx_desc, rx_list_size);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_dump_tx_desc(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        bus_size_t              tx_list_size;
        int                     error, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                printf("%s tx desc:%d\n", device_get_nameunit(sc->dev),
                       RE_TX_BUF_NUM);

                tx_list_size = sc->re_desc.tx_desc_tag->common.maxsize;
                re_dump_desc((void*)sc->re_desc.tx_desc, tx_list_size);

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_pcie_phy(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, i, n, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                printf("%s pcie phy:\n", device_get_nameunit(sc->dev));

                max = 31;
                for (n=0; n<max;) {
                        printf("\n0x%02x:\t", n);

                        for (i = 0; i < 8 && n < max; i++, n++)
                                printf("%04x ", re_ephy_read(sc, n));
                }

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_extended_registers(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, i, n, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                printf("%s extended registers:\n", device_get_nameunit(sc->dev));

                max = 0x100;
                for (n=0; n<max;) {
                        printf("\n0x%02x:\t", n);

                        for (i = 0; i < 4 && n < max; i++, n+=4)
                                printf("%08x ", re_eri_read(sc, n, 4, ERIAR_ExGMAC));
                }

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_pci_registers(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, i, n, max, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        if (result == 1) {
                sc = (struct re_softc *)arg1;
                RE_LOCK(sc);

                printf("%s pci registers:\n", device_get_nameunit(sc->dev));

                max = 0x100;
                for (n=0; n<max;) {
                        printf("\n0x%03x:\t", n);

                        for (i = 0; i < 4 && n < max; i++, n+=4)
                                printf("%08x ", re_csi_read(sc, n | 0xF000));
                }

                n = 0x108;
                printf("\n0x%03x:\t%08x ", n, re_csi_read(sc, n | 0xF000));

                n = 0x110;
                printf("\n0x%03x:\t%08x ", n, re_csi_read(sc, n | 0xF000));

                switch(sc->re_type) {
                case MACFG_62:
                case MACFG_67:
                case MACFG_68:
                case MACFG_69:
                case MACFG_74:
                case MACFG_75:
                case MACFG_76:
                        n = 0x180;
                        break;
                case MACFG_70:
                case MACFG_71:
                case MACFG_72:
                case MACFG_73:
                case MACFG_82:
                case MACFG_83:
                        n = 0x214;
                        break;
                case MACFG_80:
                case MACFG_81:
                        n = 0x264;
                        break;
                case MACFG_84:
                case MACFG_85:
                case MACFG_86:
                case MACFG_87:
                        n = 0x210;
                        break;
                case MACFG_90:
                case MACFG_91:
                case MACFG_92:
                        n = 0x22C;
                        break;
                default:
                        n =  0;
                        break;
                }
                if (n > 0)
                        printf("\n0x%03x:\t%08x ", n, re_csi_other_fun_read(sc, 0, n | 0xF000));

                n = 0x70c;
                printf("\n0x%03x:\t%08x ", n, re_csi_read(sc, n | 0xF000));

                RE_UNLOCK(sc);
        }

        return (error);
}

static int
re_sysctl_msix_tbl(SYSCTL_HANDLER_ARGS)
{
        struct re_softc         *sc;
        int                     error, i, j, result;

        result = -1;
        error = sysctl_handle_int(oidp, &result, 0, req);
        if (error || req->newptr == NULL)
                return (error);

        sc = (struct re_softc *)arg1;

        if (sc->re_res_pba == NULL)
                return EPERM;

        if (result == 1) {
                RE_LOCK(sc);

                printf("%s msix table:\n", device_get_nameunit(sc->dev));

                for (i=0; i<4; i++) {
                        printf("\n0x%04x:\t", i);

                        for (j=0; j<4; j++)
                                printf("%08x ",
                                       RE_MSIX_TBL_READ_4(
                                               sc, i*0x10 + 4 * j));
                }

                RE_UNLOCK(sc);
        }

        return (error);
}

/*
* Attach the interface. Allocate softc structures, do ifmedia
* setup and ethernet/BPF attach.
*/
static int re_attach(device_t dev)
{
        /*int			s;*/
        bus_size_t		rx_list_size, tx_list_size;
        u_char			eaddr[ETHER_ADDR_LEN];
        u_int32_t		command;
        struct re_softc		*sc;
        struct ifnet		*ifp;
        int			unit, error = 0, rid;
        int     reg;
        int		msic=0, msixc=0;

        /*s = splimp();*/

        sc = device_get_softc(dev);
        unit = device_get_unit(dev);
        bzero(sc, sizeof(struct re_softc));
        RE_LOCK_INIT(sc,device_get_nameunit(dev));
        sc->dev = dev;

        sc->driver_detach = 0;

        sc->re_vendor_id  = pci_get_vendor(dev);
        sc->re_device_id = pci_get_device(dev);
        sc->re_subvendor_id = pci_get_subvendor(dev);
        sc->re_subdevice_id = pci_get_subdevice(dev);
        sc->re_revid = pci_get_revid(dev);
        pci_enable_busmaster(dev);

        /*
         * Map control/status registers.
         */
        command = pci_read_config(dev, PCIR_COMMAND, 4);
        command |= (PCIM_CMD_PORTEN|PCIM_CMD_MEMEN|PCIM_CMD_BUSMASTEREN);
        pci_write_config(dev, PCIR_COMMAND, command, 4);

        if (prefer_iomap == 0) {
                sc->re_res_id = PCIR_BAR(2);
                sc->re_res_type = SYS_RES_MEMORY;
                /* PCI NIC use different BARs. */
                if (sc->re_device_id == RT_DEVICEID_8169 || sc->re_device_id == RT_DEVICEID_8169SC)
                        sc->re_res_id = PCIR_BAR(1);
        } else {
                sc->re_res_id = PCIR_BAR(0);
                sc->re_res_type = SYS_RES_IOPORT;
        }
        sc->re_res = bus_alloc_resource(dev, sc->re_res_type, &sc->re_res_id,
                                        0, ~0, 1, RF_ACTIVE);
        if (sc->re_res == NULL && prefer_iomap == 0) {
                sc->re_res_id = PCIR_BAR(0);
                sc->re_res_type = SYS_RES_IOPORT;
                sc->re_res = bus_alloc_resource(dev, sc->re_res_type, &sc->re_res_id,
                                                0, ~0, 1, RF_ACTIVE);
        }

        if (sc->re_res == NULL) {
                device_printf(dev,"couldn't map ports/memory\n");
                error = ENXIO;
                goto fail;
        }

        if (sc->re_res_type == SYS_RES_IOPORT)
                device_printf(dev, "Using I/O Ports\n");
        else
                device_printf(dev, "Using Memory Mapping!\n");

        sc->re_btag = rman_get_bustag(sc->re_res);
        sc->re_bhandle = rman_get_bushandle(sc->re_res);

        error = re_check_mac_version(sc);

        if (error) {
                goto fail;
        }

	device_printf(dev, "%s: HWREV=0x%08x, MACFG=%d\n",
	    __func__,
            (CSR_READ_4(sc, RE_TXCFG) & 0xFCF00000),
            sc->re_type);

        re_init_software_variable(sc);

#if OS_VER >= VERSION(7,0)
        msic = pci_msi_count(dev);
        msixc = pci_msix_count(dev);
        if (pci_find_cap(dev, PCIY_EXPRESS, &reg) == 0) {
                sc->re_if_flags |= RL_FLAG_PCIE;
                sc->re_expcap = reg;
        } else {
                sc->re_if_flags &= ~RL_FLAG_PCIE;
                sc->re_expcap = 0;
        }

        //device_printf(dev, "MSI count : %d\n", msic);
        //device_printf(dev, "MSI-X count : %d\n", msixc);
        if (sc->re_hw_enable_msi_msix == FALSE) {
                msixc = 0;
                msic = 0;
        }
        if (msix_disable > 0)
                msixc = 0;
        if (msi_disable > 0)
                msic = 0;

        /* Prefer MSI-X to MSI. */
        if (msixc > 0) {
                rid = PCIR_BAR(4);
                msixc = RL_MSI_MESSAGES;
                sc->re_res_pba = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
                                                        &rid, RF_ACTIVE);
                if (sc->re_res_pba == NULL) {
                        device_printf(dev,
                                      "could not allocate MSI-X PBA resource\n");
                }
                if (sc->re_res_pba != NULL &&
                    pci_alloc_msix(dev, &msixc) == 0) {
                        if (msixc == RL_MSI_MESSAGES) {
                                device_printf(dev, "Using %d MSI-X message\n",
                                              msixc);
                                sc->re_if_flags |= RL_FLAG_MSIX;
                        } else
                                pci_release_msi(dev);
                }
                if ((sc->re_if_flags & RL_FLAG_MSIX) == 0) {
                        if (sc->re_res_pba != NULL)
                                bus_release_resource(dev, SYS_RES_MEMORY, rid,
                                                     sc->re_res_pba);
                        sc->re_res_pba = NULL;
                        msixc = 0;
                }

                if (sc->re_res_pba != NULL) {
                        sc->re_msix_tbl_tag = rman_get_bustag(sc->re_res_pba);
                        sc->re_msix_tbl_handle = rman_get_bushandle(sc->re_res_pba);
                }
        }

        /* Prefer MSI to INTx. */
        if (msixc == 0 && msic > 0) {
                msic = RL_MSI_MESSAGES;
                if (pci_alloc_msi(dev, &msic) == 0) {
                        if (msic == RL_MSI_MESSAGES) {
                                device_printf(dev, "Using %d MSI message\n",
                                              msic);
                                sc->re_if_flags |= RL_FLAG_MSI;
                        } else
                                pci_release_msi(dev);
                }
                if ((sc->re_if_flags & RL_FLAG_MSI) == 0)
                        msic = 0;
        }
#endif //OS_VER >= VERSION(7,0)

        if ((sc->re_if_flags & (RL_FLAG_MSI | RL_FLAG_MSIX)) == 0) {
                rid = 0;
                sc->re_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, 0, ~0, 1,
                                                RF_SHAREABLE | RF_ACTIVE);

                if (sc->re_irq == NULL) {
                        device_printf(dev,"couldn't map interrupt\n");
                        error = ENXIO;
                        goto fail;
                }
                device_printf(dev, "Using line-based interrupt\n");
        } else {
                rid = 1;
                sc->re_irq = bus_alloc_resource_any(dev,
                                                    SYS_RES_IRQ, &rid, RF_ACTIVE);
                if (sc->re_irq == NULL) {
                        device_printf(dev,
                                      "couldn't allocate IRQ resources for "
                                      "message %d\n", rid);
                        error = ENXIO;
                        goto fail;
                }
        }

#if OS_VER >= VERSION(7,3)
        /* Disable ASPM L0S/L1 and Clock Request. */
        if (sc->re_expcap != 0) {
                u_int32_t		cap, ctl;
                cap = pci_read_config(dev, sc->re_expcap +
                                      RE_PCIER_LINK_CAP, 2);
                if ((cap & RE_PCIEM_LINK_CAP_ASPM) != 0) {
                        ctl = pci_read_config(dev, sc->re_expcap +
                                              RE_PCIER_LINK_CTL, 2);
                        if ((ctl & 0x0103) != 0) {
                                ctl &= ~0x0103;
                                pci_write_config(dev, sc->re_expcap +
                                                 RE_PCIER_LINK_CTL, ctl, 2);
                                device_printf(dev, "ASPM disabled\n");
                        }
                } else
                        device_printf(dev, "no ASPM capability\n");
        }
#endif //OS_VER >= VERSION(7,3)

        re_init_timer(sc);

        RE_LOCK(sc);
        re_exit_oob(sc);
        re_hw_init(sc);
        RE_UNLOCK(sc);

        /*
         * Reset the adapter. Only take the lock here as it's needed in
         * order to call re_reset().
         */
        RE_LOCK(sc);
        re_reset(sc);
        RE_UNLOCK(sc);

        sc->re_unit = unit;

        if (sc->re_type == MACFG_3) {	/* Change PCI Latency time*/
                pci_write_config(dev, RE_PCI_LATENCY_TIMER, 0x40, 1);
        }

        error = bus_dma_tag_create(
#if OS_VER < VERSION(7,0)
                        NULL,
#else
                        bus_get_dma_tag(dev),		/* parent */
#endif
                        1, 0,		/* alignment, boundary */
                        BUS_SPACE_MAXADDR,		/* lowaddr */
                        BUS_SPACE_MAXADDR,		/* highaddr */
                        NULL, NULL,			/* filter, filterarg */
                        BUS_SPACE_MAXSIZE_32BIT,	/* maxsize */
                        0,				/* nsegments */
                        BUS_SPACE_MAXSIZE_32BIT,	/* maxsegsize */
                        0,				/* flags */
                        NULL, NULL,			/* lockfunc, lockarg */
                        &sc->re_parent_tag);

        rx_list_size = sizeof(union RxDesc) * (RE_RX_BUF_NUM + 1);
        error = bus_dma_tag_create(
                        sc->re_parent_tag,
                        RE_DESC_ALIGN, 0,		/* alignment, boundary */
                        BUS_SPACE_MAXADDR,		/* lowaddr */
                        BUS_SPACE_MAXADDR,		/* highaddr */
                        NULL, NULL,			/* filter, filterarg */
                        rx_list_size,				/* maxsize */
                        1,				/* nsegments */
                        rx_list_size,				/* maxsegsize */
                        0,				/* flags */
                        NULL, NULL,			/* lockfunc, lockarg */
                        &sc->re_desc.rx_desc_tag);
        if (error) {
                device_printf(dev,"bus_dma_tag_create fail\n");
                goto fail;
        }

        error = bus_dmamem_alloc(sc->re_desc.rx_desc_tag,
                                 (void**) &sc->re_desc.rx_desc,
                                 BUS_DMA_WAITOK|BUS_DMA_COHERENT|BUS_DMA_ZERO,
                                 &sc->re_desc.rx_desc_dmamap);
        if (error) {
                device_printf(dev,"bus_dmamem_alloc fail\n");
                goto fail;
        }

        tx_list_size = sizeof(union TxDesc) * (RE_TX_BUF_NUM + 1);
        error = bus_dma_tag_create(
                        sc->re_parent_tag,
                        RE_DESC_ALIGN, 0,		/* alignment, boundary */
                        BUS_SPACE_MAXADDR,		/* lowaddr */
                        BUS_SPACE_MAXADDR,		/* highaddr */
                        NULL, NULL,			/* filter, filterarg */
                        tx_list_size,				/* maxsize */
                        1,				/* nsegments */
                        tx_list_size,				/* maxsegsize */
                        0,				/* flags */
                        NULL, NULL,			/* lockfunc, lockarg */
                        &sc->re_desc.tx_desc_tag);
        if (error) {
                device_printf(dev,"bus_dma_tag_create fail\n");
                goto fail;
        }

        error = bus_dmamem_alloc(sc->re_desc.tx_desc_tag,
                                 (void**) &sc->re_desc.tx_desc,
                                 BUS_DMA_WAITOK|BUS_DMA_COHERENT|BUS_DMA_ZERO,
                                 &sc->re_desc.tx_desc_dmamap);

        if (error) {
                device_printf(dev,"bus_dmamem_alloc fail\n");
                goto fail;
        }

        sc->re_tx_cstag =1;
        sc->re_rx_cstag =1;

        error = re_alloc_stats(dev, sc);
        if (error)
                goto fail;
        re_add_sysctls(sc);

#if OS_VER < VERSION(6,0)
        ifp = &sc->arpcom.ac_if;
#else
        ifp = sc->re_ifp = if_alloc(IFT_ETHER);
        if (ifp == NULL) {
                device_printf(dev, "can not if_alloc()\n");
                error = ENOSPC;
                goto fail;
        }
#endif
        ifp->if_softc = sc;
#if OS_VER < VERSION(5,3)
        ifp->if_unit = unit;
        ifp->if_name = "re";
#else
        if_initname(ifp, device_get_name(dev), device_get_unit(dev));
#endif
        ifp->if_mtu = ETHERMTU;
        ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
        ifp->if_ioctl = re_ioctl;
        ifp->if_output = ether_output;
        ifp->if_start = re_start;
#if OS_VER < VERSION(7,0)
        ifp->if_watchdog = re_watchdog;
#endif
        if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
                ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
        else
                ifp->if_hwassist |= RE_CSUM_FEATURES;
        ifp->if_capabilities = IFCAP_HWCSUM | IFCAP_HWCSUM_IPV6;
        /* TSO capability setup */
        if (sc->re_if_flags & RL_FLAG_8168G_PLUS) {
                ifp->if_hwassist |= CSUM_TSO;
                ifp->if_capabilities |= IFCAP_TSO;
        }
        /* RTL8169/RTL8101E/RTL8168B not support TSO v6 */
        if (!(sc->re_if_flags & RL_FLAG_DESCV2)) {
                ifp->if_hwassist &= ~(CSUM_IP6_TSO |
                                      CSUM_TCP_IPV6 |
                                      CSUM_UDP_IPV6);
                ifp->if_capabilities &= ~(IFCAP_TSO6 | IFCAP_HWCSUM_IPV6);
        }
        ifp->if_init = re_init;
        /* VLAN capability setup */
        ifp->if_capabilities |= IFCAP_VLAN_MTU | IFCAP_VLAN_HWTAGGING;
        /* LRO capability setup */
        ifp->if_capabilities |= IFCAP_LRO;

        /* Enable WOL if PM is supported. */
        if (pci_find_cap(sc->dev, PCIY_PMG, &reg) == 0)
                ifp->if_capabilities |= IFCAP_WOL;
        ifp->if_capenable = ifp->if_capabilities;
        ifp->if_capenable &= ~(IFCAP_WOL_UCAST | IFCAP_WOL_MCAST);
        /*
         * Default disable ipv6 tso.
         */
        ifp->if_hwassist &= ~CSUM_IP6_TSO;
        ifp->if_capenable &= ~IFCAP_TSO6;

        /* Not enable LRO for OS version lower than 11.0 */
#if OS_VER < VERSION(11,0)
        ifp->if_capenable &= ~IFCAP_LRO;
#endif
        /* Get station address. */
        RE_LOCK(sc);
        re_get_hw_mac_address(sc, eaddr);
        RE_UNLOCK(sc);

        /*
         * A RealTek chip was detected. Inform the world.
         */
        device_printf(dev,"version:%s\n", RE_VERSION);
        device_printf(dev,"Ethernet address: %6D\n", eaddr, ":");
        if (HW_DASH_SUPPORT_DASH(sc)) {
                device_printf(dev,"DASH status: %s\n", sc->re_dash?"enabled":"disabled");
                if (sc->re_dash)
                        device_printf(dev,"DASH FW: 0x%08x\n", sc->re_dash_fw_ver);
        }
        printf("\nThis product is covered by one or more of the following patents: \
           \nUS6,570,884, US6,115,776, and US6,327,625.\n");

#if OS_VER < VERSION(6,0)
        bcopy(eaddr, (char *)&sc->arpcom.ac_enaddr, ETHER_ADDR_LEN);
#endif
        bcopy(eaddr, (char *)&sc->org_mac_addr, ETHER_ADDR_LEN);

        RE_LOCK(sc);
        re_phy_power_up(dev);
        re_hw_phy_config(sc);
        re_clrwol(sc);

        set_rxbufsize(sc);
        error =re_alloc_buf(sc);

        if (error) {
                RE_UNLOCK(sc);
                goto fail;
        }

        /* Init descriptors. */
        re_var_init(sc);

        RE_UNLOCK(sc);

        switch(sc->re_device_id) {
        case RT_DEVICEID_8126:
                ifp->if_baudrate = 50000000000;
                break;
        case RT_DEVICEID_8125:
        case RT_DEVICEID_3000:
                ifp->if_baudrate = 25000000000;
                break;
        case RT_DEVICEID_8169:
        case RT_DEVICEID_8169SC:
        case RT_DEVICEID_8168:
        case RT_DEVICEID_8161:
        case RT_DEVICEID_8162:
                ifp->if_baudrate = 1000000000;
                break;
        default:
                ifp->if_baudrate = 100000000;
                break;
        }
        IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
        ifp->if_snd.ifq_drv_maxlen = IFQ_MAXLEN;
        IFQ_SET_READY(&ifp->if_snd);

        switch (sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                sc->ifmedia_upd = re_ifmedia_upd_8125;
                sc->ifmedia_sts = re_ifmedia_sts_8125;
                sc->intr = re_intr_8125;
                sc->int_task = re_int_task_8125;
                sc->int_task_poll = re_int_task_8125_poll;
                sc->hw_start_unlock = re_hw_start_unlock_8125;
                break;
        default:
                sc->ifmedia_upd = re_ifmedia_upd;
                sc->ifmedia_sts = re_ifmedia_sts;
                sc->intr = re_intr;
                sc->int_task = re_int_task;
                sc->int_task_poll = re_int_task_poll;
                sc->hw_start_unlock = re_hw_start_unlock;
                break;
        }

        error = re_config_soft_lro(sc);

        if (error)
                goto fail;

        /*
        * Specify the media types supported by this adapter and register
        * callbacks to update media and link information
        */
        ifmedia_init(&sc->media, IFM_IMASK, sc->ifmedia_upd, sc->ifmedia_sts);
        ifmedia_add(&sc->media, IFM_ETHER | IFM_10_T, 0, NULL);
        ifmedia_add(&sc->media, IFM_ETHER | IFM_10_T | IFM_FDX, 0, NULL);
        ifmedia_add(&sc->media, IFM_ETHER | IFM_100_TX, 0, NULL);
        ifmedia_add(&sc->media, IFM_ETHER | IFM_100_TX | IFM_FDX, 0, NULL);
        switch(sc->re_device_id) {
        case RT_DEVICEID_8125:
        case RT_DEVICEID_3000:
        case RT_DEVICEID_8126:
        case RT_DEVICEID_8169:
        case RT_DEVICEID_8169SC:
        case RT_DEVICEID_8168:
        case RT_DEVICEID_8161:
        case RT_DEVICEID_8162:
                ifmedia_add(&sc->media, IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
                break;
        default:
                break;
        }
        switch(sc->re_device_id) {
        case RT_DEVICEID_8126:
                ifmedia_add(&sc->media, IFM_ETHER | IFM_5000_T | IFM_FDX, 0, NULL);
        /* FALLTHROUGH */
        case RT_DEVICEID_8125:
        case RT_DEVICEID_3000:
                ifmedia_add(&sc->media, IFM_ETHER | IFM_2500_T | IFM_FDX, 0, NULL);
                break;
        default:
                break;
        }
        ifmedia_add(&sc->media, IFM_ETHER | IFM_AUTO, 0, NULL);
        ifmedia_set(&sc->media, IFM_ETHER | IFM_AUTO);
        sc->media.ifm_media = IFM_ETHER | IFM_AUTO;

#if OS_VER >= VERSION(13,0)
        NET_TASK_INIT(&sc->re_inttask, 0, sc->int_task, sc);
        NET_TASK_INIT(&sc->re_inttask_poll, 0, sc->int_task_poll, sc);
#elif OS_VER>=VERSION(7,0)
        TASK_INIT(&sc->re_inttask, 0, sc->int_task, sc);
        TASK_INIT(&sc->re_inttask_poll, 0, sc->int_task_poll, sc);
#endif

#if OS_VER>=VERSION(7,0)
        sc->re_tq = taskqueue_create_fast("re_taskq", M_WAITOK,
                                          taskqueue_thread_enqueue, &sc->re_tq);
        if (sc->re_tq == NULL) {
                error = ENOMEM;
                goto fail_intr;
        }
        error = taskqueue_start_threads(&sc->re_tq, 1, PI_NET, "%s taskq",
                                        device_get_nameunit(dev));
        if (error) goto fail_intr;
#endif

#if OS_VER < VERSION(7,0)
        error = bus_setup_intr(dev, sc->re_irq, INTR_TYPE_NET,
                               sc->intr, sc, &sc->re_intrhand);
#else
        error = bus_setup_intr(dev, sc->re_irq, INTR_TYPE_NET|INTR_MPSAFE,
                               sc->intr, NULL, sc, &sc->re_intrhand);
#endif

        if (error) goto fail_intr;

        RE_LOCK(sc);
        sc->ifmedia_upd(ifp);
        RE_UNLOCK(sc);

        /*
         * Call MI attach routine.
         */
        /*#if OS_VER < VERSION(5, 1)*/
#if OS_VER < VERSION(4,9)
        ether_ifattach(ifp, ETHER_BPF_SUPPORTED);
#else
        ether_ifattach(ifp, eaddr);
#endif

fail_intr:
        if (error) {
                device_printf(dev, "couldn't set up interrupt handler\n");
#if OS_VER < VERSION(4,9)
                ether_ifdetach(ifp, ETHER_BPF_SUPPORTED);
#else
                ether_ifdetach(ifp);
#endif
        }

fail:
        if (error)
                re_detach(dev);

        return(error);
}

static int re_detach(device_t dev)
{
        struct re_softc		*sc;
        struct ifnet		*ifp;
        /*int			s;*/
        int			i;
        int			rid;

        /*s = splimp();*/

        sc = device_get_softc(dev);

        ifp = RE_GET_IFNET(sc);

        re_free_soft_lro(sc);

        /* These should only be active if attach succeeded */
        if (device_is_attached(dev)) {
                RE_LOCK(sc);
                re_stop(sc);
                RE_UNLOCK(sc);
#if OS_VER < VERSION(4,9)
                ether_ifdetach(ifp, ETHER_BPF_SUPPORTED);
#else
                ether_ifdetach(ifp);
#endif
        }

        if (HW_DASH_SUPPORT_DASH(sc) && sc->re_res) {
                RE_LOCK(sc);
                re_driver_stop(sc);
                RE_UNLOCK(sc);
        }

#if OS_VER>=VERSION(7,0)
        if (sc->re_tq) {
                taskqueue_drain(sc->re_tq, &sc->re_inttask);
                taskqueue_drain(sc->re_tq, &sc->re_inttask_poll);
                taskqueue_free(sc->re_tq);
        }
#endif

        bus_generic_detach(dev);

        sc->driver_detach = 1;

        if (sc->re_intrhand)
                bus_teardown_intr(dev, sc->re_irq, sc->re_intrhand);

#if OS_VER>=VERSION(6,0)
        if (ifp)
                if_free(ifp);
#endif

        if ((sc->re_if_flags & (RL_FLAG_MSI | RL_FLAG_MSIX)) == 0)
                rid = 0;
        else
                rid = 1;
        if (sc->re_irq) {
                bus_release_resource(dev, SYS_RES_IRQ, rid, sc->re_irq);
                sc->re_irq = NULL;
        }
        if ((sc->re_if_flags & (RL_FLAG_MSI | RL_FLAG_MSIX)) != 0)
                pci_release_msi(dev);
        if (sc->re_res_pba) {
                rid = PCIR_BAR(4);
                bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->re_res_pba);
        }
        if (sc->re_res)
                bus_release_resource(dev, sc->re_res_type, sc->re_res_id, sc->re_res);

        if (HW_DASH_SUPPORT_TYPE_3(sc) && sc->re_dash)
                bus_space_unmap(sc->re_cmac_tag, sc->re_mapped_cmac_handle, RE_REGS_SIZE);

        if (sc->re_desc.re_rx_mtag) {
                for (i = 0; i < RE_RX_BUF_NUM; i++) {
                        if (sc->re_desc.rx_buf[i]!=NULL) {
                                bus_dmamap_sync(sc->re_desc.re_rx_mtag,
                                                sc->re_desc.re_rx_dmamap[i],
                                                BUS_DMASYNC_POSTREAD);
                                bus_dmamap_unload(sc->re_desc.re_rx_mtag,
                                                  sc->re_desc.re_rx_dmamap[i]);
                                bus_dmamap_destroy(sc->re_desc.re_rx_mtag,
                                                   sc->re_desc.re_rx_dmamap[i]);
                                m_freem(sc->re_desc.rx_buf[i]);
                                sc->re_desc.rx_buf[i] =NULL;
                        }
                }
                bus_dma_tag_destroy(sc->re_desc.re_rx_mtag);
                sc->re_desc.re_rx_mtag =0;
        }

        if (sc->re_desc.re_tx_mtag) {
                for (i = 0; i < RE_TX_BUF_NUM; i++) {
                        bus_dmamap_destroy(sc->re_desc.re_tx_mtag,
                                           sc->re_desc.re_tx_dmamap[i]);
                }
                bus_dma_tag_destroy(sc->re_desc.re_tx_mtag);
                sc->re_desc.re_tx_mtag =0;
        }

        if (sc->re_desc.rx_desc_tag) {
                bus_dmamap_sync(sc->re_desc.rx_desc_tag,
                                sc->re_desc.rx_desc_dmamap,
                                BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);
                bus_dmamap_unload(sc->re_desc.rx_desc_tag,
                                  sc->re_desc.rx_desc_dmamap);
                bus_dmamem_free(sc->re_desc.rx_desc_tag,
                                sc->re_desc.rx_desc,
                                sc->re_desc.rx_desc_dmamap);
                bus_dma_tag_destroy(sc->re_desc.rx_desc_tag);
        }

        if (sc->re_desc.tx_desc_tag) {
                bus_dmamap_sync(sc->re_desc.tx_desc_tag,
                                sc->re_desc.tx_desc_dmamap,
                                BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);
                bus_dmamap_unload(sc->re_desc.tx_desc_tag,
                                  sc->re_desc.tx_desc_dmamap);
                bus_dmamem_free(sc->re_desc.tx_desc_tag,
                                sc->re_desc.tx_desc,
                                sc->re_desc.tx_desc_dmamap);
                bus_dma_tag_destroy(sc->re_desc.tx_desc_tag);
        }

        /* Unload and free the stats buffer and map */

        if (sc->re_tally.re_stag) {
                if (sc->re_tally.re_stats_addr)
                        bus_dmamap_unload(sc->re_tally.re_stag,
                                          sc->re_tally.re_smap);
                if (sc->re_tally.re_stats)
                        bus_dmamem_free(sc->re_tally.re_stag,
                                        sc->re_tally.re_stats, sc->re_tally.re_smap);
                bus_dma_tag_destroy(sc->re_tally.re_stag);
        }

        if (sc->re_parent_tag) {
                bus_dma_tag_destroy(sc->re_parent_tag);
        }

        /*splx(s);*/
        RE_LOCK_DESTROY(sc);

        return(0);
}

static void
re_link_state_change(struct ifnet *ifp, int link_state)
{
#if OS_VER>=VERSION(6,0)
        if_link_state_change(ifp, link_state);
#else
        ifp->if_link_state = link_state
#endif
}

/*
  * Device suspend routine.  Stop the interface and save some PCI
  * settings in case the BIOS doesn't restore them properly on
  * resume.
  */
static int
re_suspend(device_t dev)
{
        struct re_softc         *sc;
        struct ifnet            *ifp;

        sc = device_get_softc(dev);
        RE_LOCK(sc);
        ifp = RE_GET_IFNET(sc);
        sc->re_link_chg_det = 0;
        sc->phy_reg_anlpar = re_get_phy_lp_ability(sc);
        re_stop(sc);
        re_hw_d3_para(sc);
        re_setwol(sc);
        if (HW_DASH_SUPPORT_DASH(sc))
                re_driver_stop(sc);
        sc->suspended = 1;
        sc->link_state = LINK_STATE_UNKNOWN;
        re_link_state_change(ifp, sc->link_state);
        sc->prohibit_access_reg = 1;
        RE_UNLOCK(sc);

        return (0);
}

/*
 * Device resume routine.  Restore some PCI settings in case the BIOS
 * doesn't, re-enable busmastering, and restart the interface if
 * appropriate.
 */
static int
re_resume(device_t dev)
{
        struct re_softc         *sc;
        struct ifnet            *ifp;

        sc = device_get_softc(dev);

        RE_LOCK(sc);

        ifp = RE_GET_IFNET(sc);

        sc->prohibit_access_reg = 0;

        re_exit_oob(sc);

        re_hw_init(sc);

        re_reset(sc);

        re_phy_power_up(dev);

        re_hw_phy_config(sc);

        /*
         * Clear WOL matching such that normal Rx filtering
         * wouldn't interfere with WOL patterns.
         */
        re_clrwol(sc);

        RE_UNLOCK(sc);

        RE_LOCK(sc);
        sc->ifmedia_upd(ifp);
        sc->suspended = 0;
        if (ifp->if_flags & IFF_UP) {
                sc->re_link_chg_det = 1;
                re_start_timer(sc);
        }
        RE_UNLOCK(sc);

        return (0);
}

static void
re_clear_set_ephy_bit(
        struct re_softc *sc,
        u_int8_t   addr,
        u_int16_t   clearmask,
        u_int16_t   setmask
)
{
        u_int16_t EphyValue;

        EphyValue = re_ephy_read(sc, addr);
        EphyValue &= ~clearmask;
        EphyValue |= setmask;
        re_ephy_write(sc, addr, EphyValue);
}

static void
re_clear_ephy_bit(
        struct re_softc *sc,
        u_int8_t   addr,
        u_int16_t   mask
)
{
        re_clear_set_ephy_bit(sc,
                              addr,
                              mask,
                              0
                             );
}

static void
re_set_ephy_bit(
        struct re_softc *sc,
        u_int8_t   addr,
        u_int16_t   mask
)
{
        re_clear_set_ephy_bit(sc,
                              addr,
                              0,
                              mask
                             );
}

static void
re_set_offset70f(struct re_softc *sc, u_int8_t setting)
{
        //Set PCI configuration space offset 0x79 to setting
        u_int32_t data32;

        data32 = re_csi_read(sc, 0x870c);
        data32 &= 0x00FFFFFF;
        data32 |= (setting << 24);
        re_csi_write(sc, 0x870c, data32);
}

static void
re_set_offset79(struct re_softc *sc, u_int8_t setting)
{
        //Set PCI configuration space offset 0x79 to setting
        u_int8_t data8;

        if (config_soc_lan)
                return;

        setting &= 0x70;
        data8 = pci_read_config(sc->dev, 0x79, 1);
        data8 &= ~0x70;
        data8 |= setting;
        pci_write_config(sc->dev, 0x79, data8, 1);
}

/*
 * Stop all chip I/O so that the kernel's probe routines don't
 * get confused by errant DMAs when rebooting.
 */
static int re_shutdown(device_t dev)	/* The same with re_stop(sc) */
{
        struct re_softc		*sc;

        sc = device_get_softc(dev);

        RE_LOCK(sc);
        sc->re_link_chg_det = 0;
        sc->phy_reg_anlpar = re_get_phy_lp_ability(sc);
        re_stop(sc);
        RE_UNLOCK(sc);

        RE_LOCK(sc);
        re_hw_d3_para(sc);
        if (s5wol == 0) {
                re_phy_power_down(dev);
        } else {
                struct ifnet            *ifp;
                ifp = RE_GET_IFNET(sc);
                ifp->if_capenable = IFCAP_WOL_MAGIC;
                re_setwol(sc);
        }

        if (HW_DASH_SUPPORT_DASH(sc))
                re_driver_stop(sc);
        RE_UNLOCK(sc);

        return 0;
}

static void re_set_eee_lpi_timer(struct re_softc *sc)
{
        struct ifnet		*ifp;

        ifp = RE_GET_IFNET(sc);

        switch (sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                re_mac_ocp_write(sc, RE_EEE_TXIDLE_TIMER_8168, ifp->if_mtu + ETHER_HDR_LEN + 0x20);
                break;
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                CSR_WRITE_2(sc, RE_EEE_TXIDLE_TIMER_8125, ifp->if_mtu + ETHER_HDR_LEN + 0x20);
                break;
        default:
                break;
        }
}

static void re_set_pfm_patch(struct re_softc *sc, bool enable)
{
        if (!sc->RequiredPfmPatch)
                goto exit;

        if (enable) {
                re_set_mac_ocp_bit(sc, 0xD3F0, BIT_0);
                re_set_mac_ocp_bit(sc, 0xD3F2, BIT_0);
                re_set_mac_ocp_bit(sc, 0xE85A, BIT_6);
        } else {
                re_clear_mac_ocp_bit(sc, 0xD3F0, BIT_0);
                re_clear_mac_ocp_bit(sc, 0xD3F2, BIT_0);
                re_clear_mac_ocp_bit(sc, 0xE85A, BIT_6);
        }

exit:
        return;
}

static void re_hw_start_unlock(struct re_softc *sc)
{
        struct ifnet		*ifp;
        u_int32_t		macver;
        u_int8_t		data8;
        u_int16_t		data16 = 0;
        u_int32_t		Data32;

        ifp = RE_GET_IFNET(sc);

        /* Init descriptors. */
        re_var_init(sc);

        re_enable_cfg9346_write(sc);

        switch(sc->re_type) {
        case MACFG_36:
        case MACFG_37:
        case MACFG_38:
        case MACFG_39:
        case MACFG_42:
        case MACFG_43:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
        case MACFG_54:
        case MACFG_55:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                _re_enable_aspm_clkreq_lock(sc, 0);
                re_enable_force_clkreq(sc, 0);
                break;
        }

        /*disable Link Down Power Saving(non-LDPS)*/
        /*CSR_WRITE_1(sc, RE_LDPS, 0x05);*/
        /*ldps= CSR_READ_1(sc, RE_LDPS);*/

        re_set_eee_lpi_timer(sc);

        CSR_WRITE_2(sc, RE_CPlusCmd, 0x2060);
        if (interrupt_mitigation)
                CSR_WRITE_2(sc, RE_IntrMitigate, 0x5f51);
        else
                CSR_WRITE_2(sc, RE_IntrMitigate, 0x0000);

        CSR_WRITE_1(sc, RE_MTPS, 0x3f);

        if (sc->re_device_id == RT_DEVICEID_8169 || sc->re_device_id == RT_DEVICEID_8169SC) {
                //do nothing
        } else {
                /* Set the initial TX configuration.*/
                CSR_WRITE_4(sc, RE_TXCFG, RE_TXCFG_CONFIG);
        }

        macver = CSR_READ_4(sc, RE_TXCFG) & 0xFC800000;
        if (macver == 0x00800000 || macver == 0x04000000 || macver == 0x10000000) {
                CSR_WRITE_2(sc, RE_CPlusCmd, 0x0063| ((sc->re_type == MACFG_3 && sc->re_8169_MacVersion==1) ? 0x4008:0));
        } else if (macver == 0x18000000 || macver == 0x98000000) {
                CSR_WRITE_2(sc, RE_CPlusCmd, 0x0068);
                CSR_WRITE_2(sc, RE_IntrMitigate, 0x0000);
        } else if (macver == 0x30000000) {
                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                if (ifp->if_mtu > ETHERMTU) {
                        data8 = pci_read_config(sc->dev, 0x69, 1);
                        data8 &= ~0x70;
                        data8 |= 0x28;
                        pci_write_config(sc->dev, 0x69, data8, 1);
                } else {
                        data8 = pci_read_config(sc->dev, 0x69, 1);
                        data8 &= ~0x70;
                        data8 |= 0x58;
                        pci_write_config(sc->dev, 0x69, data8, 1);
                }
        } else if (macver == 0x38000000) {
                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                if (ifp->if_mtu > ETHERMTU) {
                        data8 = pci_read_config(sc->dev, 0x69, 1);
                        data8 &= ~0x70;
                        data8 |= 0x28;
                        pci_write_config(sc->dev, 0x69, data8, 1);
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | BIT_0);
                } else {
                        data8 = pci_read_config(sc->dev, 0x69, 1);
                        data8 &= ~0x70;
                        data8 |= 0x58;
                        pci_write_config(sc->dev, 0x69, data8, 1);
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~ BIT_0);
                }
        } else if (macver == 0x34000000 || macver == 0xB4000000) {
                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
        } else if (macver == 0x34800000 || macver == 0x24800000) {
                if (pci_read_config(sc->dev, 0x81, 1) == 1) {
                        CSR_WRITE_1(sc, RE_DBG_reg, 0x98);
                        CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | 0x80);
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | 0x04);
                        pci_write_config(sc->dev, 0x81, 1, 1);
                }

                re_set_offset79(sc, 0x40);

                /*set configuration space offset 0x70f to 0x3f*/
                re_set_offset70f(sc, 0x3F);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
                if (sc->re_type == MACFG_14) {
                        CSR_WRITE_1(sc,RE_CFG1, 0x0f);

                        re_ephy_write(sc, 0x03, 0xC2F9);
                } else if (sc->re_type == MACFG_15) {
                        CSR_WRITE_1(sc,RE_CFG1, 0x0f);

                        re_ephy_write(sc, 0x01, 0x6FE5);
                        re_ephy_write(sc, 0x03, 0x07D9);
                } else if (sc->re_type == MACFG_17) {
                        re_ephy_write(sc, 0x06, 0xAF35);
                } else if (sc->re_type == MACFG_18) {
                        CSR_WRITE_1(sc, 0xF5, CSR_READ_1(sc, 0xF5)|0x04);
                        re_ephy_write(sc, 0x19, 0xEC90);
                        re_ephy_write(sc, 0x01, 0x6FE5);
                        re_ephy_write(sc, 0x03, 0x05D9);
                        re_ephy_write(sc, 0x06, 0xAF35);
                } else if (sc->re_type == MACFG_19) {
                        if (pci_read_config(sc->dev, 0x80, 1)&3) {
                                re_ephy_write(sc, 0x02, 0x011F);
                        }
                        CSR_WRITE_1(sc, 0xF4, CSR_READ_1(sc, 0xF4)|0x08);
                        CSR_WRITE_1(sc, 0xF5, CSR_READ_1(sc, 0xF5)|0x04);
                        re_ephy_write(sc, 0x19, 0xEC90);
                        re_ephy_write(sc, 0x01, 0x6FE5);
                        re_ephy_write(sc, 0x03, 0x05D9);
                        re_ephy_write(sc, 0x06, 0xAF35);
                }
        } else if (macver == 0x3C000000) {
                //disable clock request.
                pci_write_config(sc->dev, 0x81, 0, 1);

                /*set configuration space offset 0x70f to 0x27*/
                re_set_offset70f(sc, 0x27);

                CSR_WRITE_1(sc, RE_CFG1, CSR_READ_1(sc, RE_CFG1)|0x10);
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
                if (sc->re_type == MACFG_24) {
                        /*set mac register offset 0xd1 to 0xf8*/
                        CSR_WRITE_1(sc, RE_DBG_reg, 0xF8);

                        data16 = re_ephy_read(sc, 0x02) & ~0x1800;
                        data16 |= 0x1000;
                        re_ephy_write(sc, 0x02, data16);

                        data16 = re_ephy_read(sc, 0x03) | 0x0002;
                        re_ephy_write(sc, 0x03, data16);

                        data16 = re_ephy_read(sc, 0x06) & ~0x0080;
                        re_ephy_write(sc, 0x06, data16);

                        if (ifp->if_mtu > ETHERMTU) {
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1 << 1)); //Jumbo_en1

                                re_set_offset79(sc, 0x20);
                                ifp->if_capenable &= ~IFCAP_HWCSUM;
                                CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
                                ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                        } else {
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1 << 1)); //Jumbo_en1
                                re_set_offset79(sc, 0x40);
                                if (sc->re_tx_cstag) {
                                        ifp->if_capenable |= IFCAP_TXCSUM;
                                        if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
                                                ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
                                        else
                                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                                }
                                if (sc->re_rx_cstag) {
                                        ifp->if_capenable |= IFCAP_RXCSUM;
                                        CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
                                }
                        }
                } else if (sc->re_type == MACFG_25) {
                        data16 = re_ephy_read(sc, 0x01) | 0x0001;
                        re_ephy_write(sc, 0x01, data16);

                        data16 = re_ephy_read(sc, 0x03) & ~0x0620;
                        data16 |= 0x0220;
                        re_ephy_write(sc, 0x03, data16);

                        if (ifp->if_mtu > ETHERMTU) {
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

                                re_set_offset79(sc, 0x20);
                                ifp->if_capenable &= ~IFCAP_HWCSUM;
                                CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
                                ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                        } else {
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
                                re_set_offset79(sc, 0x40);
                                if (sc->re_tx_cstag) {
                                        ifp->if_capenable |= IFCAP_TXCSUM;
                                        if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
                                                ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
                                        else
                                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                                }
                                if (sc->re_rx_cstag) {
                                        ifp->if_capenable |= IFCAP_RXCSUM;
                                        CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
                                }
                        }
                } else if (sc->re_type == MACFG_26) {
                        if (ifp->if_mtu > ETHERMTU) {
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

                                re_set_offset79(sc, 0x20);
                                ifp->if_capenable &= ~IFCAP_HWCSUM;
                                CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
                                ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                        } else {
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
                                re_set_offset79(sc, 0x40);
                                if (sc->re_tx_cstag) {
                                        ifp->if_capenable |= IFCAP_TXCSUM;
                                        if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
                                                ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
                                        else
                                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                                }
                                if (sc->re_rx_cstag) {
                                        ifp->if_capenable |= IFCAP_RXCSUM;
                                        CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
                                }
                        }
                }
        } else if (macver == 0x3C800000) {
                //disable clock request.
                pci_write_config(sc->dev, 0x81, 0x00, 1);

                /*set configuration space offset 0x70f to 0x27*/
                re_set_offset70f(sc, 0x27);

                re_eri_write(sc, 0x1EC, 1, 0x07, ERIAR_ASF);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);
                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);
                if (sc->re_type == MACFG_28)
                        CSR_WRITE_1(sc, 0xD1, 0x20);

                if (ifp->if_mtu > ETHERMTU) {
                        CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

                        re_set_offset79(sc, 0x20);
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
                        re_set_offset79(sc, 0x40);
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                                CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
                        }
                }
        } else if (macver == 0x28000000) {
                //disable clock request.
                pci_write_config(sc->dev, 0x81, 0x00, 1);

                /*set configuration space offset 0x70f to 0x13*/
                re_set_offset70f(sc, 0x27);

                CSR_WRITE_1(sc, RE_TDFNR, 0x8);

                CSR_WRITE_1(sc, 0xD1, CSR_READ_1(sc, 0xD1) | 0x02);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);
                CSR_WRITE_1(sc, RE_DBG_reg, CSR_READ_1(sc, RE_DBG_reg)|0x82);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                if (ifp->if_mtu > ETHERMTU) {
                        CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

                        re_set_offset79(sc, 0x20);
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
                        re_set_offset79(sc, 0x40);
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                                CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
                        }
                }

                if (sc->re_type == MACFG_31) {
                        CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~(1<<4));

                        re_ephy_write(sc, 0x01, 0x7C7F);
                        re_ephy_write(sc, 0x02, 0x011F);
                        re_ephy_write(sc, 0x06, 0xB271);
                        re_ephy_write(sc, 0x07, 0xCE00);
                } else if (sc->re_type == MACFG_32) {
                        re_ephy_write(sc, 0x01, 0x7C7D);
                        re_ephy_write(sc, 0x02, 0x091F);
                        re_ephy_write(sc, 0x03, 0xC5BA);
                        re_ephy_write(sc, 0x06, 0xB279);
                        re_ephy_write(sc, 0x07, 0xAF00);
                        re_ephy_write(sc, 0x1E, 0xB8EB);
                } else if (sc->re_type == MACFG_33) {
                        CSR_WRITE_1(sc, RE_CFG1, CSR_READ_1(sc, RE_CFG1)|0x10);

                        re_ephy_write(sc, 0x01, 0x6C7F);
                        re_ephy_write(sc, 0x02, 0x011F);
                        re_clear_set_ephy_bit(sc,
                                              0x03,
                                              0xFFF0,
                                              0x01B0
                                             );
                        re_ephy_write(sc, 0x1A, 0x0546);
                        re_ephy_write(sc, 0x1C, 0x80C4);
                        re_ephy_write(sc, 0x1D, 0x78E5);
                        re_ephy_write(sc, 0x0A, 0x8100);
                }
        } else if (macver == 0x28800000) {
                /* disable clock request. */
                pci_write_config(sc->dev, 0x81, 0x00, 1);

                /*set configuration space offset 0x70f to 0x17*/
                re_set_offset70f(sc, 0x27);

                CSR_WRITE_1(sc, RE_TDFNR, 0x8);
                if (sc->re_dash &&
                    (sc->re_type == MACFG_63 || sc->re_type == MACFG_64))
                        CSR_WRITE_1(sc, RE_TDFNR, 0x1);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);
                CSR_WRITE_1(sc, RE_DBG_reg, CSR_READ_1(sc, RE_DBG_reg)|0x82);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                if (ifp->if_mtu > ETHERMTU) {
                        CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2); //Jumbo_en0
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | (1<<1)); //Jumbo_en1

                        re_set_offset79(sc, 0x20);
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2); //Jumbo_en0
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~(1<<1)); //Jumbo_en1
                        re_set_offset79(sc, 0x40);
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                                CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
                        }
                }

                if (sc->re_type == MACFG_65 || sc->re_type == MACFG_66) {
                        re_set_ephy_bit(sc, 0x0B, (BIT_3 | BIT_6));

                        re_clear_set_ephy_bit(sc,
                                              0x19,
                                              BIT_5,
                                              (BIT_4 | BIT_6)
                                             );

                        re_clear_set_ephy_bit(sc,
                                              0x0C,
                                              BIT_8,
                                              BIT_5
                                             );

                        re_clear_ephy_bit(sc, 0x10, (BIT_2));
                }
        } else if (macver == 0x2C000000) {
                /* disable clock request. */
                pci_write_config(sc->dev, 0x81, 0x00, 1);

                /*set configuration space offset 0x70f to 0x20*/
                re_set_offset70f(sc, 0x27);

                CSR_WRITE_1(sc, 0xF3, CSR_READ_1(sc, 0xF3)|0x20);
                CSR_WRITE_1(sc, 0xF3, CSR_READ_1(sc, 0xF3)& ~0x20);

                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0)|0xC0);
                CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1)|0x73);
                CSR_WRITE_1(sc, RE_CFG5, (CSR_READ_1(sc, RE_CFG5)& ~0x08));
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_1(sc, RE_TDFNR, 0x8);

                if (sc->re_type == MACFG_36 || sc->re_type == MACFG_37) {
                        /* set EPHY registers */
                        data16 = re_ephy_read(sc, 0x00) & ~0x0200;
                        data16 |= 0x0100;
                        re_ephy_write(sc, 0x00, data16);

                        data16 = re_ephy_read(sc, 0x00);
                        data16 |= 0x0004;
                        re_ephy_write(sc, 0x00, data16);

                        data16 = re_ephy_read(sc, 0x06) & ~0x0002;
                        data16 |= 0x0001;
                        re_ephy_write(sc, 0x06, data16);

                        data16 = re_ephy_read(sc, 0x06);
                        data16 |= 0x0030;
                        re_ephy_write(sc, 0x06, data16);

                        data16 = re_ephy_read(sc, 0x07);
                        data16 |= 0x2000;
                        re_ephy_write(sc, 0x07, data16);

                        data16 = re_ephy_read(sc, 0x00);
                        data16 |= 0x0020;
                        re_ephy_write(sc, 0x00, data16);

                        data16 = re_ephy_read(sc, 0x03) & ~0x5800;
                        data16 |= 0x2000;
                        re_ephy_write(sc, 0x03, data16);

                        data16 = re_ephy_read(sc, 0x03);
                        data16 |= 0x0001;
                        re_ephy_write(sc, 0x03, data16);

                        data16 = re_ephy_read(sc, 0x01) & ~0x0800;
                        data16 |= 0x1000;
                        re_ephy_write(sc, 0x01, data16);

                        data16 = re_ephy_read(sc, 0x07);
                        data16 |= 0x4000;
                        re_ephy_write(sc, 0x07, data16);

                        data16 = re_ephy_read(sc, 0x1E);
                        data16 |= 0x2000;
                        re_ephy_write(sc, 0x1E, data16);

                        re_ephy_write(sc, 0x19, 0xFE6C);

                        data16 = re_ephy_read(sc, 0x0A);
                        data16 |= 0x0040;
                        re_ephy_write(sc, 0x0A, data16);

                        if (ifp->if_mtu > ETHERMTU) {
                                CSR_WRITE_1 (sc, RE_MTPS, 0x24);
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) | BIT_2);
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) |0x01);
                                re_set_offset79(sc, 0x20);
                                ifp->if_capenable &= ~IFCAP_HWCSUM;
                                CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) & ~RL_RxChkSum);
                                ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                        } else {
                                CSR_WRITE_1 (sc, RE_MTPS, 0x0c);
                                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_2);
                                CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) & ~0x01);
                                re_set_offset79(sc, 0x40);

                                if (sc->re_tx_cstag) {
                                        ifp->if_capenable |= IFCAP_TXCSUM;
                                        ifp->if_hwassist |= RE_CSUM_FEATURES;
                                }
                                if (sc->re_rx_cstag) {
                                        ifp->if_capenable |= IFCAP_RXCSUM;
                                        CSR_WRITE_2 (sc, RE_CPlusCmd,CSR_READ_2(sc, RE_CPlusCmd) |RL_RxChkSum);
                                }
                        }
                }
        } else if (macver == 0x2C800000) {
                /* disable clock request. */
                pci_write_config(sc->dev, 0x81, 0x00, 1);

                /*set configuration space offset 0x70f to 0x27*/
                re_set_offset70f(sc, 0x27);

                re_set_offset79(sc, 0x40);

                CSR_WRITE_1(sc, RE_TDFNR, 0x8);

                re_eri_write(sc, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                re_eri_write(sc, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
                re_eri_write(sc, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
                re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
                Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 |= BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0xD4, 4, ERIAR_ExGMAC);
                Data32 |= (BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                re_eri_write(sc, 0xD4, 4, Data32, ERIAR_ExGMAC);
                if (sc ->re_type == MACFG_39) {
                        Data32 = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                        Data32 |= BIT_4;
                        re_eri_write(sc, 0x1B0, 4, Data32, ERIAR_ExGMAC);
                        re_eri_write(sc, 0xCC, 4, 0x00000050, ERIAR_ExGMAC);
                        re_eri_write(sc, 0xD0, 4, 0x07ff0060, ERIAR_ExGMAC);
                }

                CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) |BIT_7);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                if (sc ->re_type == MACFG_38) {
                        CSR_WRITE_4(sc, 0xB0, 0xEE480010);
                        CSR_WRITE_1(sc, 0x1A, CSR_READ_1(sc, 0x1A) & ~(BIT_2 |BIT_3));
                        re_eri_write(sc, 0x1DC, 1, 0x64, ERIAR_ExGMAC);

                        re_ephy_write(sc, 0x06, 0xF020);
                        re_ephy_write(sc, 0x07, 0x01FF);
                        re_ephy_write(sc, 0x00, 0x5027);
                        re_ephy_write(sc, 0x01, 0x0003);
                        re_ephy_write(sc, 0x02, 0x2D16);
                        re_ephy_write(sc, 0x03, 0x6D49);
                        re_ephy_write(sc, 0x08, 0x0006);
                        re_ephy_write(sc, 0x0A, 0x00C8);
                }

                data16 = re_ephy_read(sc, 0x09);
                data16 |= BIT_7;
                re_ephy_write(sc, 0x09, data16);

                data16 = re_ephy_read(sc, 0x19);
                data16 |= (BIT_2 | BIT_5 | BIT_9);
                re_ephy_write(sc, 0x19, data16);

                re_set_ephy_bit(sc, 0x00, BIT_3);
                re_clear_set_ephy_bit(sc,
                                      0x0C,
                                      (BIT_13|BIT_12|BIT_11|BIT_10|BIT_8|BIT_7|BIT_6|BIT_5|BIT_4),
                                      BIT_9
                                     );

                CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_5);
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

                if (ifp->if_mtu > ETHERMTU)
                        CSR_WRITE_1 (sc, RE_MTPS, 0x27);
                ifp->if_capenable &= ~IFCAP_HWCSUM;
                ifp->if_hwassist &= ~RE_CSUM_FEATURES;
        } else if (macver == 0x24000000) {
                if (pci_read_config(sc->dev, 0x81, 1)==1) {
                        CSR_WRITE_1(sc, RE_DBG_reg, 0x98);
                        CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | 0x80);
                        CSR_WRITE_1(sc, RE_CFG4, CSR_READ_1(sc, RE_CFG4) | 0x04);
                        pci_write_config(sc->dev, 0x81, 1, 1);
                }
                re_set_offset79(sc, 0x40);

                /*set configuration space offset 0x70f to 0x3F*/
                re_set_offset70f(sc, 0x3F);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                re_ephy_write(sc, 0x06, 0xAF25);
                re_ephy_write(sc, 0x07, 0x8E68);
        } else if (macver == 0x40800000) {
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | 0x80);
                CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1) | 0x28);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) | 0x0C);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | 0x40);
                CSR_WRITE_2(sc, 0xE0, CSR_READ_2(sc, 0xE0) & ~0xDF9C);
                CSR_WRITE_1(sc, 0xD1, CSR_READ_1(sc, 0xD1) | 0x02);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                if (sc->re_type == MACFG_42) {
                        /* set EPHY registers */
                        re_set_ephy_bit(sc, 0x07, BIT_14);
                        re_set_ephy_bit(sc, 0x19, BIT_9);
                        re_set_ephy_bit(sc, 0x19, BIT_5);
                        re_set_ephy_bit(sc, 0x1E, BIT_13);
                        re_set_ephy_bit(sc, 0x03, BIT_0);
                        re_set_ephy_bit(sc, 0x19, BIT_8);
                        re_set_ephy_bit(sc, 0x19, BIT_2);
                        re_set_ephy_bit(sc, 0x0A, BIT_5);
                        re_set_ephy_bit(sc, 0x05, BIT_13);
                }
                if (sc->re_type == MACFG_43) {
                        re_set_ephy_bit(sc, 0x07, BIT_14);
                        re_set_ephy_bit(sc, 0x19, BIT_9);
                        re_set_ephy_bit(sc, 0x19, BIT_5);
                        re_set_ephy_bit(sc, 0x1E, BIT_13);
                        re_set_ephy_bit(sc, 0x03, BIT_0);
                        re_set_ephy_bit(sc, 0x19, BIT_8);
                        re_set_ephy_bit(sc, 0x19, BIT_2);
                        re_set_ephy_bit(sc, 0x0A, BIT_5);
                        re_set_ephy_bit(sc, 0x1E, BIT_15);
                        re_set_ephy_bit(sc, 0x05, BIT_13);
                }
        } else if (macver == 0x44000000) {
                CSR_WRITE_2(sc, 0xE0, CSR_READ_2(sc, 0xE0) & ~0xDF9C);

                re_eri_write(sc, 0xC8, 4, 0x00000002, ERIAR_ExGMAC);
                re_eri_write(sc, 0xE8, 4, 0x00000006, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0xD4, 4, ERIAR_ExGMAC);
                Data32 |= BIT_11 | BIT_10;
                re_eri_write(sc, 0xD4, 4, Data32, ERIAR_ExGMAC);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                /* set EPHY registers */
                re_ephy_write(sc, 0x19, 0xFF64);

                if (ifp->if_mtu > ETHERMTU)
                        CSR_WRITE_1 (sc, RE_MTPS, 0x27);
        } else if (macver == 0x48000000) {
                /*set configuration space offset 0x70f to 0x27*/
                re_set_offset70f(sc, 0x27);

                re_set_offset79(sc, 0x40);

                CSR_WRITE_1(sc, RE_TDFNR, 0x8);

                Data32 = re_eri_read(sc, 0xD4, 4, ERIAR_ExGMAC);
                Data32 |= (BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                re_eri_write(sc, 0xD4, 4, Data32, ERIAR_ExGMAC);
                re_eri_write(sc, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                re_eri_write(sc, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
                re_eri_write(sc, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
                re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
                Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 |= BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                Data32 |= BIT_4;
                re_eri_write(sc, 0x1B0, 4, Data32, ERIAR_ExGMAC);
                re_eri_write(sc, 0xCC, 4, 0x00000050, ERIAR_ExGMAC);
                re_eri_write(sc, 0xD0, 4, 0x00000060, ERIAR_ExGMAC);
                Data32 = re_eri_read(sc, 0x1D0, 4, ERIAR_ExGMAC);
                Data32 |= BIT_4;
                re_eri_write(sc, 0x1D0, 4, Data32, ERIAR_ExGMAC);

                CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) | BIT_7);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                if (sc->re_type == MACFG_50) {
                        data16 = re_ephy_read(sc, 0x06);
                        data16 &= ~(BIT_7 | BIT_6);
                        data16 |= BIT_5;
                        re_ephy_write(sc, 0x06, data16);

                        data16 = re_ephy_read(sc, 0x08);
                        data16 &= ~BIT_0;
                        data16 |= BIT_1;
                        re_ephy_write(sc, 0x08, data16);
                }

                data16 = re_ephy_read(sc, 0x09);
                data16 |= BIT_7;
                re_ephy_write(sc, 0x09, data16);

                data16 = re_ephy_read(sc, 0x19);
                data16 |= (BIT_2 | BIT_5 | BIT_9);
                re_ephy_write(sc, 0x19, data16);

                re_set_ephy_bit(sc, 0x00, BIT_3);
                re_clear_set_ephy_bit(sc,
                                      0x0C,
                                      (BIT_13|BIT_12|BIT_11|BIT_10|BIT_8|BIT_7|BIT_6|BIT_5|BIT_4),
                                      BIT_9
                                     );

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

                if (ifp->if_mtu > ETHERMTU)
                        CSR_WRITE_1 (sc, RE_MTPS, 0x27);

                if (ifp->if_mtu > ETHERMTU) {
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                        }
                }
        } else if (macver == 0x48800000) {
                /*set configuration space offset 0x70f to 0x27*/
                re_set_offset70f(sc, 0x27);

                re_set_offset79(sc, 0x40);

                CSR_WRITE_1(sc, RE_TDFNR, 0x8);

                Data32 = re_eri_read(sc, 0xD4, 4, ERIAR_ExGMAC);
                Data32 |= BIT_11 | BIT_10;
                re_eri_write(sc, 0xD4, 4, Data32, ERIAR_ExGMAC);
                re_eri_write(sc, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                re_eri_write(sc, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
                re_eri_write(sc, 0xC8, 4, 0x00100002, ERIAR_ExGMAC);
                re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
                Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 |= BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                Data32 |= BIT_4;
                re_eri_write(sc, 0x1B0, 4, Data32, ERIAR_ExGMAC);
                re_eri_write(sc, 0xCC, 4, 0x00000050, ERIAR_ExGMAC);
                re_eri_write(sc, 0xD0, 4, 0x00000060, ERIAR_ExGMAC);
                Data32 = re_eri_read(sc, 0x1D0, 4, ERIAR_ExGMAC);
                Data32 |= BIT_4;
                re_eri_write(sc, 0x1D0, 4, Data32, ERIAR_ExGMAC);

                CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) | BIT_7);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                //CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                data16 = re_ephy_read(sc, 0x06);
                data16 &= ~(BIT_7 | BIT_6);
                data16 |= BIT_5;
                re_ephy_write(sc, 0x06, data16);

                re_ephy_write(sc, 0x0f, 0x5200);

                data16 = re_ephy_read(sc, 0x1e);
                data16 |= BIT_14;
                re_ephy_write(sc, 0x1e, data16);

                data16 = re_ephy_read(sc, 0x19);
                data16 |= (BIT_2 | BIT_5 | BIT_9);
                re_ephy_write(sc, 0x19, data16);

                re_set_ephy_bit(sc, 0x00, BIT_3);
                re_clear_set_ephy_bit(sc,
                                      0x0C,
                                      (BIT_13|BIT_12|BIT_11|BIT_10|BIT_8|BIT_7|BIT_6|BIT_5|BIT_4),
                                      BIT_9
                                     );

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

                if (ifp->if_mtu > ETHERMTU)
                        CSR_WRITE_1 (sc, RE_MTPS, 0x27);

                if (ifp->if_mtu > ETHERMTU) {
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                        }
                }
        } else if (macver == 0x44800000) {
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | 0x80);
                CSR_WRITE_1(sc, 0xF1, CSR_READ_1(sc, 0xF1) | 0x28);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) | 0x0C);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | 0x40);
                CSR_WRITE_2(sc, 0xE0, CSR_READ_2(sc, 0xE0) & ~0xDF9C);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);
        } else if (macver == 0x4C000000 || macver == 0x50800000 ||
                   macver == 0x5C800000 || macver == 0x54000000 ||
                   macver == 0x6C000000) {
                CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_5);

                if (sc->re_type == MACFG_59) {
                        re_mac_ocp_write(sc, 0xD3C0, 0x0B00);
                        re_mac_ocp_write(sc, 0xD3C2, 0x0000);
                }

                if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                    sc->re_type == MACFG_74 || sc->re_type == MACFG_75 ||
                    sc->re_type == MACFG_76) {
                        re_mac_ocp_write(sc, 0xD400, re_mac_ocp_read(sc, 0xD400) & ~(BIT_0));

                        data16 = re_mac_ocp_read(sc, 0xE63E);
                        data16 &= ~(BIT_3 | BIT_2 | BIT_1);
                        re_mac_ocp_write(sc, 0xE63E, data16);
                        data16 |= (BIT_0);
                        re_mac_ocp_write(sc, 0xE63E, data16);
                        data16 &= ~(BIT_0);
                        re_mac_ocp_write(sc, 0xE63E, data16);
                        re_mac_ocp_write(sc, 0xC094, 0x0);
                        re_mac_ocp_write(sc, 0xC09E, 0x0);

                        re_mac_ocp_write(sc, 0xE098, 0x0AA2);
                }

                /*set configuration space offset 0x70f to 0x17*/
                re_set_offset70f(sc, 0x27);

                re_set_offset79(sc, 0x40);

                CSR_WRITE_1(sc, RE_TDFNR, 0x4);

                if (sc->re_type == MACFG_56 || sc->re_type == MACFG_57) {
                        Data32 = re_csi_read(sc, 0x2710);
                        Data32 &=0xFFFF0FFF;
                        Data32 |= (0x04 << 12);
                        re_csi_write(sc, 0x2710, Data32);
                }

                Data32 = re_eri_read(sc, 0xD4, 4, ERIAR_ExGMAC);
                Data32 |= (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                re_eri_write(sc, 0xD4, 4, Data32, ERIAR_ExGMAC);

                if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                    sc->re_type == MACFG_74 || sc->re_type == MACFG_75 ||
                    sc->re_type == MACFG_76) {
                        Data32 = re_eri_read(sc, 0xDC, 4, ERIAR_ExGMAC);
                        Data32 |= (BIT_2| BIT_3 | BIT_4);
                        re_eri_write(sc, 0xDC, 4, Data32, ERIAR_ExGMAC);
                }

                re_eri_write(sc, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
                re_eri_write(sc, 0xCC, 1, 0x38, ERIAR_ExGMAC);
                re_eri_write(sc, 0xD0, 1, 0x48, ERIAR_ExGMAC);
                re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);

                if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                    sc->re_type == MACFG_74 || sc->re_type == MACFG_75 ||
                    sc->re_type == MACFG_76) {
                        re_mac_ocp_write(sc, 0xE054, 0x0000);

                        Data32 = re_eri_read(sc, 0x5F0, 4, ERIAR_ExGMAC);
                        Data32 &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                        re_eri_write(sc, 0x5F0, 4, Data32, ERIAR_ExGMAC);
                } else {
                        re_eri_write(sc, 0x5F0, 2, 0x4F87, ERIAR_ExGMAC);
                }

                Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 |= BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);

                if (sc->re_type == MACFG_74 || sc->re_type == MACFG_75)
                        re_set_mac_ocp_bit(sc, 0xD438, (BIT_1 | BIT_0));

                Data32 = re_eri_read(sc, 0x2FC, 4, ERIAR_ExGMAC);
                Data32 &= ~(BIT_0 | BIT_1 | BIT_2);
                Data32 |= (BIT_0);
                re_eri_write(sc, 0x2FC, 4, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_12;
                re_eri_write(sc, 0x1B0, 4, Data32, ERIAR_ExGMAC);

                CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) | BIT_7);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                if (sc->re_type == MACFG_56 || sc->re_type == MACFG_57) {
                        re_clear_ephy_bit(sc, 0x00, BIT_3);
                        re_clear_set_ephy_bit(sc,
                                              0x0C,
                                              (BIT_13|BIT_12|BIT_10|BIT_9|BIT_8|BIT_7|BIT_6|BIT_4),
                                              (BIT_11|BIT_5)
                                             );
                        re_set_ephy_bit(sc, 0x1E, BIT_0);
                        re_clear_ephy_bit(sc, 0x19, BIT_15);
                }  else if (sc->re_type == MACFG_58) {
                        re_set_ephy_bit(sc, 0x00, (BIT_3));
                        re_clear_set_ephy_bit(sc,
                                              0x0C,
                                              (BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4),
                                              BIT_9
                                             );
                }  else if (sc->re_type == MACFG_59) {
                        re_clear_ephy_bit(sc, 0x00, BIT_3);
                        re_clear_set_ephy_bit(sc,
                                              0x0C,
                                              (BIT_13 | BIT_12 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_4),
                                              (BIT_5 | BIT_11)
                                             );

                        re_set_ephy_bit(sc, 0x1E, BIT_0);
                        re_clear_ephy_bit(sc, 0x19, BIT_15);
                        re_ephy_write(sc, 0x19, 0x7C00);
                        re_ephy_write(sc, 0x1E, 0x20EB);
                        re_ephy_write(sc, 0x0D, 0x1666);
                        re_ephy_write(sc, 0x00, 0x10A3);

                        re_ephy_write(sc, 0x06, 0xF050);

                        re_set_ephy_bit(sc, 0x04, BIT_4);
                        re_clear_ephy_bit(sc, 0x1D, BIT_14);
                } else if (sc->re_type == MACFG_60) {
                        re_clear_ephy_bit(sc, 0x00, BIT_3);
                        re_clear_set_ephy_bit(sc,
                                              0x0C,
                                              (BIT_13 | BIT_12 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_4),
                                              (BIT_5 | BIT_11)
                                             );
                        re_set_ephy_bit(sc, 0x1E, BIT_0);
                        re_clear_ephy_bit(sc, 0x19, BIT_15);

                        re_clear_ephy_bit(sc, 0x19, (BIT_5 | BIT_0));

                        re_set_ephy_bit(sc, 0x1E, BIT_13);
                        re_clear_ephy_bit(sc, 0x0D, BIT_8);
                        re_set_ephy_bit(sc, 0x0D, BIT_9);
                        re_set_ephy_bit(sc, 0x00, BIT_7);

                        re_set_ephy_bit(sc, 0x06, BIT_4);

                        re_set_ephy_bit(sc, 0x04, BIT_4);
                        re_set_ephy_bit(sc, 0x1D, BIT_14);
                } else if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                           sc->re_type == MACFG_76) {
                        re_clear_ephy_bit(sc, 0x1E, BIT_11);

                        re_set_ephy_bit(sc, 0x1E, BIT_0);
                        re_set_ephy_bit(sc, 0x1D, BIT_11);

                        re_ephy_write(sc, 0x05, 0x2089);
                        re_ephy_write(sc, 0x06, 0x5881);

                        re_ephy_write(sc, 0x04, 0x854A);
                        re_ephy_write(sc, 0x01, 0x068B);
                } else if (sc->re_type == MACFG_74) {
                        re_clear_mac_ocp_bit(sc, 0xD438, BIT_2);

                        re_clear_ephy_bit(sc, 0x24, BIT_9);
                        re_clear_mac_ocp_bit(sc, 0xDE28, (BIT_1 | BIT_0));

                        re_set_mac_ocp_bit(sc, 0xD438, BIT_2);
                } else if (sc->re_type == MACFG_75) {
                        re_clear_mac_ocp_bit(sc, 0xD438, BIT_2);

                        re_clear_mac_ocp_bit(sc, 0xDE28, (BIT_1 | BIT_0));

                        re_set_mac_ocp_bit(sc, 0xD438, BIT_2);
                }

                if (sc->re_type == MACFG_60) {
                        data16 = re_mac_ocp_read(sc, 0xD3C0);
                        data16 &= 0xF000;
                        data16 |= 0x0FFF;
                        re_mac_ocp_write(sc, 0xD3C0, data16);

                        data16 = re_mac_ocp_read(sc, 0xD3C2);
                        data16 &= 0xFF00;
                        re_mac_ocp_write(sc, 0xD3C2, data16);

                        data16 = re_mac_ocp_read(sc, 0xD3C4);
                        data16 |= (BIT_0);
                        re_mac_ocp_write(sc, 0xD3C4, data16);
                } else if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                           sc->re_type == MACFG_76) {
                        if (sc->RequireAdjustUpsTxLinkPulseTiming) {
                                data16 = re_mac_ocp_read(sc, 0xD412);
                                data16 &= ~(0x0FFF);
                                data16 |= sc->SwrCnt1msIni;
                                re_mac_ocp_write(sc, 0xD412, data16);
                        }

                        data16 = re_mac_ocp_read(sc, 0xE056);
                        data16 &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
                        re_mac_ocp_write(sc, 0xE056, data16);

                        data16 = re_mac_ocp_read(sc, 0xE052);
                        data16 &= ~(BIT_15 | BIT_14 | BIT_13 | BIT_3);
                        data16 |= BIT_15;
                        re_mac_ocp_write(sc, 0xE052, data16);

                        data16 = re_mac_ocp_read(sc, 0xD420);
                        data16 &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                        data16 |= 0x45F;
                        re_mac_ocp_write(sc, 0xD420, data16);

                        data16 = re_mac_ocp_read(sc, 0xE0D6);
                        data16 &= ~(BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                        data16 |= 0x17F;
                        re_mac_ocp_write(sc, 0xE0D6, data16);
                }

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~BIT_3);

                if (ifp->if_mtu > ETHERMTU)
                        CSR_WRITE_1 (sc, RE_MTPS, 0x27);

                if (sc->re_type == MACFG_56 || sc->re_type == MACFG_57 ||
                    sc->re_type == MACFG_58 || sc->re_type == MACFG_59) {
                        re_mac_ocp_write(sc, 0xC140, 0xFFFF);
                } else if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                           sc->re_type == MACFG_74 || sc->re_type == MACFG_75 ||
                           sc->re_type == MACFG_76) {
                        re_mac_ocp_write(sc, 0xC140, 0xFFFF);
                        re_mac_ocp_write(sc, 0xC142, 0xFFFF);
                }

                if (ifp->if_mtu > ETHERMTU) {
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                        }
                }
        } else if (macver == 0x50000000) {
                /*set configuration space offset 0x70f to 0x17*/
                re_set_offset70f(sc, 0x27);

                re_set_offset79(sc, 0x40);

                Data32 = re_eri_read(sc, 0xD4, 4, ERIAR_ExGMAC);
                Data32 |= BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12;
                re_eri_write(sc, 0xD4, 4, Data32, ERIAR_ExGMAC);

                re_eri_write(sc, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
                re_eri_write(sc, 0xCC, 1, 0x2F, ERIAR_ExGMAC);
                re_eri_write(sc, 0xD0, 1, 0x5F, ERIAR_ExGMAC);
                re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);

                if (sc->re_type == MACFG_62 || sc->re_type == MACFG_67) {
                        OOB_mutex_lock(sc);
                        re_eri_write(sc, 0x5F0, 4, 0x4F87, ERIAR_ExGMAC);
                        OOB_mutex_unlock(sc);
                }

                Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 |= BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0x2FC, 4, ERIAR_ExGMAC);
                Data32 &= ~(BIT_0 | BIT_1 | BIT_2);
                Data32 |= (BIT_0 | BIT_1);
                re_eri_write(sc, 0x2FC, 4, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_12;
                re_eri_write(sc, 0x1B0, 4, Data32, ERIAR_ExGMAC);

                CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) | BIT_7);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                if (sc->re_type == MACFG_61) {
                        re_ephy_write(sc, 0x00, 0x10AB);
                        re_ephy_write(sc, 0x06, 0xF030);
                        re_ephy_write(sc, 0x08, 0x2006);
                        re_ephy_write(sc, 0x0D, 0x1666);
                        re_clear_ephy_bit(sc, 0x0C, (BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4));
                }  else if (sc->re_type == MACFG_62) {
                        re_ephy_write(sc, 0x00, 0x10A3);
                        re_ephy_write(sc, 0x19, 0xFC00);
                        re_ephy_write(sc, 0x1E, 0x20EA);
                } else if (sc->re_type == MACFG_67) {
                        re_ephy_write(sc, 0x00, 0x10AB);
                        re_ephy_write(sc, 0x19, 0xFC00);
                        re_ephy_write(sc, 0x1E, 0x20EB);
                        re_ephy_write(sc, 0x0D, 0x1666);
                        re_clear_ephy_bit(sc, 0x0B, BIT_0);
                        re_set_ephy_bit(sc, 0x1D, BIT_14);
                        re_clear_set_ephy_bit(sc,
                                              0x0C,
                                              BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_8 | BIT_7 | BIT_6 | BIT_5,
                                              BIT_9 | BIT_4
                                             );
                }

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);

                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~BIT_3);

                if (ifp->if_mtu > ETHERMTU)
                        CSR_WRITE_1 (sc, RE_MTPS, 0x27);

                if (sc->re_type == MACFG_67) {
                        data16 = re_mac_ocp_read(sc, 0xD3E2);
                        data16 &= 0xF000;
                        data16 |= 0xAFD;
                        re_mac_ocp_write(sc, 0xD3E2, data16);

                        data16 = re_mac_ocp_read(sc, 0xD3E4);
                        data16 &= 0xFF00;
                        re_mac_ocp_write(sc, 0xD3E4, data16);

                        data16 = re_mac_ocp_read(sc, 0xE860);
                        data16 |= BIT_7;
                        re_mac_ocp_write(sc, 0xE860, data16);
                }

                re_mac_ocp_write(sc, 0xC140, 0xFFFF);
                re_mac_ocp_write(sc, 0xC142, 0xFFFF);

                if (ifp->if_mtu > ETHERMTU) {
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                        }
                }
        } else if (macver == 0x54800000) {
                re_mac_ocp_write(sc, 0xE098, 0xC302);

                re_mac_ocp_write(sc, 0xD400, re_mac_ocp_read(sc, 0xD400) & ~(BIT_0));

                if (sc->RequireAdjustUpsTxLinkPulseTiming) {
                        data16 = re_mac_ocp_read(sc, 0xD412);
                        data16 &= ~(0x0FFF);
                        data16 |= sc->SwrCnt1msIni;
                        re_mac_ocp_write(sc, 0xD412, data16);
                }

                data16 = re_mac_ocp_read(sc, 0xE056);
                data16 &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
                if (sc->HwPkgDet == 0x0F)
                        data16 |= (BIT_6 | BIT_5 | BIT_4);
                re_mac_ocp_write(sc, 0xE056, data16);
                if (FALSE == HW_SUPP_SERDES_PHY(sc))
                        re_mac_ocp_write(sc, 0xEA80, 0x0003);
                else
                        re_mac_ocp_write(sc, 0xEA80, 0x0000);

                OOB_mutex_lock(sc);
                data16 = re_mac_ocp_read(sc, 0xE052);
                data16 &= ~(BIT_3 | BIT_0);
                if (sc->HwPkgDet == 0x0F)
                        data16 |= BIT_0;
                re_mac_ocp_write(sc, 0xE052, data16);
                OOB_mutex_unlock(sc);

                data16 = re_mac_ocp_read(sc, 0xD420);
                data16 &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                data16 |= 0x45F;
                re_mac_ocp_write(sc, 0xD420, data16);

                CSR_WRITE_1(sc, RE_TDFNR, 0x4);

                data16 = re_mac_ocp_read(sc, 0xE63E);
                data16 &= ~(BIT_3 | BIT_2 | BIT_1);
                re_mac_ocp_write(sc, 0xE63E, data16);
                data16 |= (BIT_0);
                re_mac_ocp_write(sc, 0xE63E, data16);
                data16 &= ~(BIT_0);
                re_mac_ocp_write(sc, 0xE63E, data16);
                re_mac_ocp_write(sc, 0xC094, 0x0);
                re_mac_ocp_write(sc, 0xC09E, 0x0);

                /*set configuration space offset 0x70f to 0x27*/
                re_set_offset70f(sc, 0x27);

                re_set_offset79(sc, 0x40);

                Data32 = re_eri_read(sc, 0xD4, 4, ERIAR_ExGMAC);
                Data32 |= BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12;
                if (sc->re_type == MACFG_71 || sc->re_type == MACFG_72 ||
                    sc->re_type == MACFG_73)
                        Data32 |= BIT_4;
                re_eri_write(sc, 0xD4, 4, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0xDC, 4, ERIAR_ExGMAC);
                Data32 |= (BIT_2| BIT_3);
                re_eri_write(sc, 0xDC, 4, Data32, ERIAR_ExGMAC);

                re_eri_write(sc, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
                re_eri_write(sc, 0xCC, 1, 0x2F, ERIAR_ExGMAC);
                re_eri_write(sc, 0xD0, 1, 0x5F, ERIAR_ExGMAC);
                re_eri_write(sc, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);

                OOB_mutex_lock(sc);
                if (sc->HwPkgDet == 0x0F)
                        re_eri_write(sc, 0x5F0, 2, 0x4F00, ERIAR_ExGMAC);
                else
                        re_eri_write(sc, 0x5F0, 2, 0x4000, ERIAR_ExGMAC);
                OOB_mutex_unlock(sc);

                Data32 = re_eri_read(sc, 0xdc, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);
                Data32 |= BIT_0;
                re_eri_write(sc, 0xdc, 1, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0x2FC, 4, ERIAR_ExGMAC);
                Data32 &= ~(BIT_0 | BIT_1);
                Data32 |= (BIT_0);
                re_eri_write(sc, 0x2FC, 4, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                Data32 &= ~BIT_12;
                re_eri_write(sc, 0x1B0, 4, Data32, ERIAR_ExGMAC);

                Data32 = re_eri_read(sc, 0x1D0, 1, ERIAR_ExGMAC);
                Data32 &= ~BIT_1;
                re_eri_write(sc, 0x1D0, 1, Data32, ERIAR_ExGMAC);

                re_eri_write(sc, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
                re_eri_write(sc, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);

                CSR_WRITE_4(sc, RE_TXCFG, CSR_READ_4(sc, RE_TXCFG) | BIT_7);
                CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) & ~BIT_7);
                CSR_WRITE_1(sc, 0x1B, CSR_READ_1(sc, 0x1B) & ~0x07);

                CSR_WRITE_2 (sc, RE_CPlusCmd, 0x2060);

                re_clear_set_ephy_bit(sc,
                                      0x19,
                                      BIT_6,
                                      (BIT_12| BIT_8)
                                     );
                re_clear_set_ephy_bit(sc,
                                      0x59,
                                      BIT_6,
                                      (BIT_12| BIT_8)
                                     );
                re_clear_ephy_bit(sc, 0x0C, BIT_4);
                re_clear_ephy_bit(sc, 0x4C, BIT_4);
                re_clear_ephy_bit(sc, 0x0B, BIT_0);

                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_0);

                if (FALSE == HW_SUPP_SERDES_PHY(sc)) {
                        CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_6);
                        CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) | BIT_6);
                        CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_7);
                } else {
                        CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) & ~BIT_6);
                        CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~BIT_6);
                        CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) & ~BIT_7);
                }

                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~BIT_3);

                if (ifp->if_mtu > ETHERMTU)
                        CSR_WRITE_1 (sc, RE_MTPS, 0x27);

                re_mac_ocp_write(sc, 0xC140, 0xFFFF);
                re_mac_ocp_write(sc, 0xC142, 0xFFFF);

                if (ifp->if_mtu > ETHERMTU) {
                        ifp->if_capenable &= ~IFCAP_HWCSUM;
                        ifp->if_hwassist &= ~RE_CSUM_FEATURES;
                } else {
                        if (sc->re_tx_cstag) {
                                ifp->if_capenable |= IFCAP_TXCSUM;
                                ifp->if_hwassist |= RE_CSUM_FEATURES;
                        }
                        if (sc->re_rx_cstag) {
                                ifp->if_capenable |= IFCAP_RXCSUM;
                        }
                }
        }

        if (!((sc->re_if_flags & RL_FLAG_DESCV2) &&
              (sc->re_if_flags & RL_FLAG_8168G_PLUS)))
                ifp->if_hwassist &= ~(CSUM_TCP_IPV6 | CSUM_UDP_IPV6);

        //clear io_rdy_l23
        switch (sc->re_type) {
        case MACFG_42:
        case MACFG_43:
        case MACFG_52:
        case MACFG_53:
        case MACFG_54:
        case MACFG_55:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                CSR_WRITE_1(sc, RE_CFG3, CSR_READ_1(sc, RE_CFG3) & ~BIT_1);
                break;
        }

        switch(sc->re_type) {
        case MACFG_36:
        case MACFG_37:
        case MACFG_38:
        case MACFG_39:
        case MACFG_42:
        case MACFG_43:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
        case MACFG_54:
        case MACFG_55:
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                _re_enable_aspm_clkreq_lock(sc, 1);
                re_enable_force_clkreq(sc, 0);
                break;
        }

        //clear wol
        re_clrwol(sc);

        data16 = CSR_READ_2(sc, RE_CPlusCmd);
        if ((ifp->if_capenable & IFCAP_VLAN_HWTAGGING) != 0)
                data16 |= RL_CPLUSCMD_VLANSTRIP;
        else
                data16 &= ~RL_CPLUSCMD_VLANSTRIP;

        if ((ifp->if_capenable & IFCAP_RXCSUM) != 0)
                data16 |= RL_RxChkSum;
        else
                data16 &= ~RL_RxChkSum;
        CSR_WRITE_2 (sc, RE_CPlusCmd, data16);

        re_disable_cfg9346_write(sc);
        //CSR_WRITE_1(sc, 0xec, 0x3f);

        if (sc->re_device_id == RT_DEVICEID_8169 || sc->re_device_id == RT_DEVICEID_8169SC) {
                /* Enable transmit and receive.*/
                CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_TX_ENB | RE_CMD_RX_ENB);

                /* Set the initial TX configuration.*/
                CSR_WRITE_4(sc, RE_TXCFG, RE_TXCFG_CONFIG);

                /* Set the initial RX configuration.*/
                /*
                 * Program the multicast filter, if necessary.
                 */
                re_set_rx_packet_filter(sc);
        } else {
                /* Set the initial RX configuration.*/
                /*
                 * Program the multicast filter, if necessary.
                 */
                re_set_rx_packet_filter(sc);

                /* Enable transmit and receive.*/
                CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_TX_ENB | RE_CMD_RX_ENB);
        }

        ifp->if_drv_flags |= IFF_DRV_RUNNING;
        ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

        /*
        * Enable interrupts.
        */
        CSR_WRITE_2(sc, RE_IMR, RE_INTRS);
}

static void re_init_unlock(void *xsc)  	/* Software & Hardware Initialize */
{
        struct re_softc		*sc = xsc;
        struct ifnet		*ifp;
#if OS_VER < VERSION(6,0)
        int			i;
#endif
        union {
                uint32_t align_dummy;
                u_char eaddr[ETHER_ADDR_LEN];
        } eaddr;

        ifp = RE_GET_IFNET(sc);

        /*
         * Cancel pending I/O and free all RX/TX buffers.
         */
        re_stop(sc);

        /*
        * Disable TSO if interface MTU size is greater than MSS
        * allowed in controller.
        */
        if (ifp->if_mtu > ETHERMTU) {
                ifp->if_capenable &= ~(IFCAP_TSO | IFCAP_VLAN_HWTSO);
                ifp->if_hwassist &= ~CSUM_TSO;
        }

        /* Copy MAC address on stack to align. */
#if OS_VER < VERSION(6,0)
        bcopy((char *)&sc->arpcom.ac_enaddr, eaddr.eaddr, ETHER_ADDR_LEN);
#elif OS_VER < VERSION(7,0)
        bcopy(IFP2ENADDR(ifp), eaddr.eaddr, ETHER_ADDR_LEN);
#else
        bcopy(IF_LLADDR(ifp), eaddr.eaddr, ETHER_ADDR_LEN);
#endif

        /* Init our MAC address */
        re_rar_set(sc, eaddr.eaddr);

        sc->hw_start_unlock(sc);

        return;
}


static void re_init_locked(void *xsc)  	/* Software & Hardware Initialize */
{
        struct re_softc		*sc = xsc;
        struct ifnet		*ifp;

        ifp = RE_GET_IFNET(sc);

        if (re_link_ok(sc)) {
                sc->link_state = LINK_STATE_UP;
                re_link_state_change(ifp, sc->link_state);
                re_link_on_patch(sc);
        }

        sc->re_link_chg_det = 1;
        re_start_timer(sc);
}

static void re_init(void *xsc)  	/* Software & Hardware Initialize */
{
        struct re_softc		*sc = xsc;

        RE_LOCK(sc);
        re_init_locked(sc);
        RE_UNLOCK(sc);
}

static void re_hw_start_unlock_8125(struct re_softc *sc)
{
        struct ifnet		*ifp;
        u_int32_t		macver;
        u_int16_t		data16 = 0;
        u_int8_t 		data8;

        ifp = RE_GET_IFNET(sc);

        /* Init descriptors. */
        re_var_init(sc);

        re_enable_cfg9346_write(sc);

        _re_enable_aspm_clkreq_lock(sc, 0);
        re_enable_force_clkreq(sc, 0);

        re_set_eee_lpi_timer(sc);

        CSR_WRITE_2(sc, RE_CPlusCmd, 0x2060);

        /* Set the initial TX configuration.*/
        CSR_WRITE_4(sc, RE_TXCFG, RE_TXCFG_CONFIG);

        macver = CSR_READ_4(sc, RE_TXCFG) & 0xFC800000;
        if (macver == 0x60800000 || macver == 0x64000000 ||
            macver == 0x64800000 || macver == 0x68000000 ||
            macver == 0x68800000) {
                CSR_WRITE_1(sc, RE_CFG2, CSR_READ_1(sc, RE_CFG2) | BIT_5);

                re_mac_ocp_write(sc, 0xE098, 0xC302);

                /*set configuration space offset 0x70f to 0x17*/
                if (macver == 0x64800000)
                        re_set_offset70f(sc, 0x17);
                else
                        re_set_offset70f(sc, 0x27);

                re_set_offset79(sc, 0x40);

                if (macver == 0x60800000 || macver == 0x64000000)
                        CSR_WRITE_2(sc, 0x382, 0x221B);

                CSR_WRITE_1(sc, 0x4500, 0x00);
                CSR_WRITE_2(sc, 0x4800, 0x0000);

                CSR_WRITE_1(sc, RE_CFG1, CSR_READ_1(sc, RE_CFG1) & ~0x10);

                CSR_WRITE_1(sc, 0xF2, CSR_READ_1(sc, 0xF2) & ~BIT_3);

                CSR_WRITE_1(sc, RE_TDFNR, 0x10);

                if (sc->re_type == MACFG_80) {
                        re_ephy_write(sc, 0x01, 0xA812);
                        re_ephy_write(sc, 0x09, 0x520C);
                        re_ephy_write(sc, 0x04, 0xD000);
                        re_ephy_write(sc, 0x0D, 0xF702);
                        re_ephy_write(sc, 0x0A, 0x8653);
                        re_ephy_write(sc, 0x06, 0x001E);
                        re_ephy_write(sc, 0x08, 0x3595);
                        re_ephy_write(sc, 0x20, 0x9455);
                        re_ephy_write(sc, 0x21, 0x99FF);
                        re_ephy_write(sc, 0x02, 0x6046);
                        re_ephy_write(sc, 0x29, 0xFE00);
                        re_ephy_write(sc, 0x23, 0xAB62);
                        re_clear_ephy_bit(sc, 0x24, BIT_11);

                        re_ephy_write(sc, 0x41, 0xA80C);
                        re_ephy_write(sc, 0x49, 0x520C);
                        re_ephy_write(sc, 0x44, 0xD000);
                        re_ephy_write(sc, 0x4D, 0xF702);
                        re_ephy_write(sc, 0x4A, 0x8653);
                        re_ephy_write(sc, 0x46, 0x001E);
                        re_ephy_write(sc, 0x48, 0x3595);
                        re_ephy_write(sc, 0x60, 0x9455);
                        re_ephy_write(sc, 0x61, 0x99FF);
                        re_ephy_write(sc, 0x42, 0x6046);
                        re_ephy_write(sc, 0x69, 0xFE00);
                        re_ephy_write(sc, 0x63, 0xAB62);
                        re_clear_ephy_bit(sc, 0x64, BIT_11);
                }  else if (sc->re_type == MACFG_81) {
                        re_ephy_write(sc, 0x04, 0xD000);
                        re_ephy_write(sc, 0x0A, 0x8653);
                        re_ephy_write(sc, 0x23, 0xAB66);
                        re_ephy_write(sc, 0x20, 0x9455);
                        re_ephy_write(sc, 0x21, 0x99FF);
                        re_ephy_write(sc, 0x29, 0xFE04);

                        re_ephy_write(sc, 0x44, 0xD000);
                        re_ephy_write(sc, 0x4A, 0x8653);
                        re_ephy_write(sc, 0x63, 0xAB66);
                        re_ephy_write(sc, 0x60, 0x9455);
                        re_ephy_write(sc, 0x61, 0x99FF);
                        re_ephy_write(sc, 0x69, 0xFE04);

                        re_clear_set_ephy_bit(sc,
                                              0x2A,
                                              (BIT_14 | BIT_13 | BIT_12),
                                              (BIT_13 | BIT_12)
                                             );
                        re_clear_ephy_bit(sc, 0x19, BIT_6);
                        re_set_ephy_bit(sc, 0x1B, (BIT_11 | BIT_10 | BIT_9));
                        re_clear_ephy_bit(sc, 0x1B, (BIT_14 | BIT_13 | BIT_12));
                        re_ephy_write(sc, 0x02, 0x6042);
                        re_ephy_write(sc, 0x06, 0x0014);

                        re_clear_set_ephy_bit(sc,
                                              0x6A,
                                              (BIT_14 | BIT_13 | BIT_12),
                                              (BIT_13 | BIT_12)
                                             );
                        re_clear_ephy_bit(sc, 0x59, BIT_6);
                        re_set_ephy_bit(sc, 0x5B, (BIT_11 | BIT_10 | BIT_9));
                        re_clear_ephy_bit(sc, 0x5B, (BIT_14 | BIT_13 | BIT_12));
                        re_ephy_write(sc, 0x42, 0x6042);
                        re_ephy_write(sc, 0x46, 0x0014);
                } else if (sc->re_type == MACFG_82) {
                        re_ephy_write(sc, 0x06, 0x001F);
                        re_ephy_write(sc, 0x0A, 0xB66B);
                        re_ephy_write(sc, 0x01, 0xA852);
                        re_ephy_write(sc, 0x24, 0x0008);
                        re_ephy_write(sc, 0x2F, 0x6052);
                        re_ephy_write(sc, 0x0D, 0xF716);
                        re_ephy_write(sc, 0x20, 0xD477);
                        re_ephy_write(sc, 0x21, 0x4477);
                        re_ephy_write(sc, 0x22, 0x0013);
                        re_ephy_write(sc, 0x23, 0xBB66);
                        re_ephy_write(sc, 0x0B, 0xA909);
                        re_ephy_write(sc, 0x29, 0xFF04);
                        re_ephy_write(sc, 0x1B, 0x1EA0);

                        re_ephy_write(sc, 0x46, 0x001F);
                        re_ephy_write(sc, 0x4A, 0xB66B);
                        re_ephy_write(sc, 0x41, 0xA84A);
                        re_ephy_write(sc, 0x64, 0x000C);
                        re_ephy_write(sc, 0x6F, 0x604A);
                        re_ephy_write(sc, 0x4D, 0xF716);
                        re_ephy_write(sc, 0x60, 0xD477);
                        re_ephy_write(sc, 0x61, 0x4477);
                        re_ephy_write(sc, 0x62, 0x0013);
                        re_ephy_write(sc, 0x63, 0xBB66);
                        re_ephy_write(sc, 0x4B, 0xA909);
                        re_ephy_write(sc, 0x69, 0xFF04);
                        re_ephy_write(sc, 0x5B, 0x1EA0);
                } else if (sc->re_type == MACFG_83) {
                        re_ephy_write(sc, 0x0B, 0xA908);
                        re_ephy_write(sc, 0x1E, 0x20EB);
                        re_ephy_write(sc, 0x22, 0x0023);
                        re_ephy_write(sc, 0x02, 0x60C2);
                        re_ephy_write(sc, 0x29, 0xFF00);

                        re_ephy_write(sc, 0x4B, 0xA908);
                        re_ephy_write(sc, 0x5E, 0x28EB);
                        re_ephy_write(sc, 0x62, 0x0023);
                        re_ephy_write(sc, 0x42, 0x60C2);
                        re_ephy_write(sc, 0x69, 0xFF00);
                } else if (sc->re_type == MACFG_84 || sc->re_type == MACFG_85 ||
                           sc->re_type == MACFG_86 || sc->re_type == MACFG_87) {
                        //do nothing
                }

                re_mac_ocp_write(sc, 0xC140, 0xFFFF);
                re_mac_ocp_write(sc, 0xC142, 0xFFFF);

                //old tx desc format
                data16 = re_mac_ocp_read(sc, 0xEB58);
                if (sc->re_type == MACFG_91 || sc->re_type == MACFG_92)
                        data16 &= ~(BIT_0 | BIT_1);
                else
                        data16 &= ~(BIT_0);
                re_mac_ocp_write(sc, 0xEB58, data16);

                if (sc->re_type == MACFG_84 || sc->re_type == MACFG_85 ||
                    sc->re_type == MACFG_86 || sc->re_type == MACFG_87 ||
                    sc->re_type == MACFG_91 || sc->re_type == MACFG_92)
                        CSR_WRITE_1(sc, 0xD8, CSR_READ_1(sc, 0xD8) & ~BIT_1);

                data16 = re_mac_ocp_read(sc, 0xE614);
                data16 &= ~(BIT_10 | BIT_9 | BIT_8);
                if (sc->re_type == MACFG_82 || sc->re_type == MACFG_83)
                        data16 |= (2 << 8);
                else if (sc->re_type == MACFG_90 || sc->re_type == MACFG_91 ||
                         sc->re_type == MACFG_92)
                        data16 |= (4 << 8);
                else
                        data16 |= (3 << 8);
                re_mac_ocp_write(sc, 0xE614, data16);

                data16 = re_mac_ocp_read(sc, 0xE63E);
                data16 &= ~(BIT_11 | BIT_10);
                re_mac_ocp_write(sc, 0xE63E, data16);

                data16 = re_mac_ocp_read(sc, 0xE63E);
                data16 &= ~(BIT_5 | BIT_4);
                if (sc->re_type == MACFG_80 || sc->re_type == MACFG_81 ||
                    sc->re_type == MACFG_90 || sc->re_type == MACFG_91 ||
                    sc->re_type == MACFG_92)
                        data16 |= ((0x02 & 0x03) << 4);
                re_mac_ocp_write(sc, 0xE63E, data16);

                data16 = re_mac_ocp_read(sc, 0xC0B4);
                data16 |= (BIT_3|BIT_2);
                re_mac_ocp_write(sc, 0xC0B4, data16);

                data16 = re_mac_ocp_read(sc, 0xEB6A);
                data16 &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                data16 |= (BIT_5 | BIT_4 | BIT_1 | BIT_0);
                re_mac_ocp_write(sc, 0xEB6A, data16);

                data16 = re_mac_ocp_read(sc, 0xEB50);
                data16 &= ~(BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5);
                data16 |= (BIT_6);
                re_mac_ocp_write(sc, 0xEB50, data16);

                data16 = re_mac_ocp_read(sc, 0xE056);
                data16 &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
                //data16 |= (BIT_4 | BIT_5);
                re_mac_ocp_write(sc, 0xE056, data16);

                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | BIT_7);

                data16 = re_mac_ocp_read(sc, 0xE040);
                data16 &= ~(BIT_12);
                re_mac_ocp_write(sc, 0xE040, data16);

                data16 = re_mac_ocp_read(sc, 0xEA1C);
                data16 &= ~(BIT_1 | BIT_0);
                data16 |= (BIT_0);
                re_mac_ocp_write(sc, 0xEA1C, data16);

                if (HW_DASH_SUPPORT_DASH(sc))
                        OOB_mutex_lock(sc);

                if (sc->re_type == MACFG_84 || sc->re_type == MACFG_85)
                        re_mac_ocp_write(sc, 0xE0C0, 0x4403);
                else
                        re_mac_ocp_write(sc, 0xE0C0, 0x4000);

                re_set_mac_ocp_bit(sc, 0xE052, (BIT_6 | BIT_5));
                re_clear_mac_ocp_bit(sc, 0xE052, BIT_3 | BIT_7);

                if (HW_DASH_SUPPORT_DASH(sc))
                        OOB_mutex_unlock(sc);

                data16 = re_mac_ocp_read(sc, 0xC0AC);
                data16 |= (BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12);
                if (macver == 0x60800000)
                        data16 &= ~(BIT_7);
                re_mac_ocp_write(sc, 0xC0AC, data16);

                data16 = re_mac_ocp_read(sc, 0xD430);
                data16 &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
                data16 |= 0x45F;
                re_mac_ocp_write(sc, 0xD430, data16);

                //re_mac_ocp_write(sc, 0xE0C0, 0x4F87);
                CSR_WRITE_1(sc, 0xD0, CSR_READ_1(sc, 0xD0) | (BIT_6 | BIT_7));

                if (sc->re_type == MACFG_80 || sc->re_type == MACFG_81)
                        CSR_WRITE_1(sc, 0xD3, CSR_READ_1(sc, 0xD3) | BIT_0);

                if (sc->re_type != MACFG_84 && sc->re_type != MACFG_85)
                        re_mac_ocp_write(sc, 0xE080, re_mac_ocp_read(sc, 0xE080)&~BIT_1);

                data16 = re_mac_ocp_read(sc, 0xEA1C);
                data16 &= ~(BIT_2);
                if (sc->re_type == MACFG_91 || sc->re_type == MACFG_92)
                        data16 &= ~(BIT_9 | BIT_8);
                re_mac_ocp_write(sc, 0xEA1C, data16);

                re_set_mac_ocp_bit(sc, 0xEB54, BIT_0);
                DELAY(1);
                re_clear_mac_ocp_bit(sc, 0xEB54, BIT_0);
                CSR_WRITE_2(sc, 0x1880, CSR_READ_2(sc, 0x1880) & ~(BIT_4 | BIT_5));

                if (macver == 0x60800000 || macver == 0x68000000 ||
                    macver == 0x68800000) {
                        for (int i=0xA00; i<0xB00; i+=4)
                                CSR_WRITE_4(sc, i, 0x0000);
                } else {
                        for (int i = 0xA00; i < 0xA80; i += 4)
                                CSR_WRITE_4(sc, i, 0x0000);
                }

                if (macver == 0x60800000) {
                        //do nothing
                } else {
                        data8 = CSR_READ_1(sc, RE_INT_CFG0_8125);
                        data8 &= ~(RTL8125_INT_CFG0_ENABLE_8125 |
                                   RTL8125_INT_CFG0_TIMEOUT0_BYPASS |
                                   RTL8125_INT_CFG0_MITIGATION_BYPASS);
                        if (sc->re_type == MACFG_91 || sc->re_type == MACFG_92)
                                data8 &= ~RTL8126_INT_CFG0_RDU_BYPASS;
                        if (sc->re_type == MACFG_84 || sc->re_type == MACFG_85 ||
                            sc->re_type == MACFG_86 || sc->re_type == MACFG_87)
                                data8 &= ~RTL8125_INT_CFG0_MSIX_ENTRY_NUM_MODE;
                        CSR_WRITE_1(sc, RE_INT_CFG0_8125, data8);

                        CSR_WRITE_2(sc, RE_INT_CFG1_8125, 0x0000);
                }

                if (sc->re_tx_cstag) {
                        ifp->if_capenable |= IFCAP_TXCSUM;
                        ifp->if_hwassist |= RE_CSUM_FEATURES;
                }
                if (sc->re_rx_cstag) {
                        ifp->if_capenable |= IFCAP_RXCSUM;
                }
        }

        if (sc->RequiredPfmPatch)
                re_set_pfm_patch(sc, 0);

        _re_enable_aspm_clkreq_lock(sc, 1);
        re_enable_force_clkreq(sc, 0);

        //clear wol
        re_clrwol(sc);

        //Interrupt Mitigation
        if (macver == 0x68000000 || macver == 0x68800000)
                CSR_WRITE_4(sc, 0x0A00, 0x00140014);
        else
                CSR_WRITE_4(sc, 0x0A00, 0x00630063);

        if ((ifp->if_capenable & IFCAP_VLAN_HWTAGGING) != 0)
                CSR_WRITE_4(sc, RE_RXCFG, CSR_READ_4(sc, RE_RXCFG) | (BIT_22 | BIT_23));
        else
                CSR_WRITE_4(sc, RE_RXCFG, CSR_READ_4(sc, RE_RXCFG) & ~(BIT_22 | BIT_23));

        data16 = CSR_READ_2(sc, RE_CPlusCmd);
        if ((ifp->if_capenable & IFCAP_RXCSUM) != 0)
                data16 |= RL_RxChkSum;
        else
                data16 &= ~RL_RxChkSum;
        CSR_WRITE_2 (sc, RE_CPlusCmd, data16);

        re_disable_cfg9346_write(sc);

        /* Set the initial RX configuration.*/
        /*
         * Program the multicast filter, if necessary.
         */
        re_set_rx_packet_filter(sc);

        /* Enable transmit and receive.*/
        CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_TX_ENB | RE_CMD_RX_ENB);

        ifp->if_drv_flags |= IFF_DRV_RUNNING;
        ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

        /*
        * Enable interrupts.
        */
        CSR_WRITE_4(sc, RE_IMR0_8125, RE_INTRS);
}

/*
 * Initialize the transmit descriptors.
 */
static int re_var_init(struct re_softc *sc)
{
        int			i;
        union RxDesc *rxptr;
        union TxDesc *txptr;

        sc->re_desc.rx_cur_index = 0;
        rxptr = sc->re_desc.rx_desc;
        for (i = 0; i < RE_RX_BUF_NUM; i++) {
                memset(&rxptr[i], 0, sizeof(union RxDesc));

                /* Init the RX buffer pointer register. */
                bus_dmamap_load(sc->re_desc.re_rx_mtag,
                                sc->re_desc.re_rx_dmamap[i],
                                sc->re_desc.rx_buf[i]->m_data, sc->re_rx_desc_buf_sz,
                                re_rx_dma_map_buf,
                                &rxptr[i],
                                0);
                bus_dmamap_sync(sc->re_desc.re_rx_mtag,
                                sc->re_desc.re_rx_dmamap[i],
                                BUS_DMASYNC_PREREAD);

                rxptr[i].ul[0] = htole32(sc->re_rx_desc_buf_sz);
                if (i == (RE_RX_BUF_NUM - 1))
                        rxptr[i].ul[0] |= htole32(RL_RDESC_CMD_EOR);
                rxptr[i].ul[0] |= htole32(RL_RDESC_CMD_OWN);
        }

        bus_dmamap_load(sc->re_desc.rx_desc_tag,
                        sc->re_desc.rx_desc_dmamap,
                        sc->re_desc.rx_desc,
                        sizeof(union RxDesc)*RE_RX_BUF_NUM,
                        re_dma_map_rxdesc,
                        sc,
                        0);
        bus_dmamap_sync(sc->re_desc.rx_desc_tag,
                        sc->re_desc.rx_desc_dmamap,
                        BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

        sc->re_desc.tx_cur_index = 0;
        sc->re_desc.tx_last_index = 0;
        txptr = sc->re_desc.tx_desc;
        for (i = 0; i < RE_TX_BUF_NUM; i++) {
                memset(&txptr[i], 0, sizeof(union TxDesc));
                if (i == (RE_TX_BUF_NUM - 1))
                        txptr[i].ul[0] = htole32(RL_TDESC_CMD_EOR);
        }

        bus_dmamap_load(sc->re_desc.tx_desc_tag,
                        sc->re_desc.tx_desc_dmamap,
                        sc->re_desc.tx_desc,
                        sizeof(union TxDesc) * RE_TX_BUF_NUM,
                        re_dma_map_txdesc,
                        sc,
                        0);
        bus_dmamap_sync(sc->re_desc.tx_desc_tag,
                        sc->re_desc.tx_desc_dmamap,
                        BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

        return 0;
}

static void re_reset(struct re_softc *sc)
{
        register int		i;

        re_clear_all_rx_packet_filter(sc);

        switch (sc->re_type) {
        case MACFG_3:
        case MACFG_4:
        case MACFG_5:
        case MACFG_6:
                DELAY(10000);
                break;
        case MACFG_11:
        case MACFG_12:
        case MACFG_13:
        case MACFG_14:
        case MACFG_15:
        case MACFG_16:
        case MACFG_17:
        case MACFG_18:
        case MACFG_19:
        case MACFG_21:
        case MACFG_22:
        case MACFG_23:
        case MACFG_24:
        case MACFG_25:
        case MACFG_26:
        case MACFG_27:
        case MACFG_28:
        case MACFG_31:
        case MACFG_32:
        case MACFG_33:
        case MACFG_36:
        case MACFG_37:
        case MACFG_41:
        case MACFG_42:
        case MACFG_43:
        case MACFG_54:
        case MACFG_55:
        case MACFG_63:
        case MACFG_64:
        case MACFG_65:
        case MACFG_66:
                CSR_WRITE_1(sc, RE_COMMAND, 0x8C);
                break;
        case MACFG_38:
        case MACFG_39:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
        case MACFG_53:
                CSR_WRITE_1(sc, RE_COMMAND, 0x8C);
                while (!(CSR_READ_4(sc,RE_TXCFG) & BIT_11)) DELAY(100);
                break;
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                DELAY(2000);
                break;
        default:
                DELAY(10000);
                break;
        }
        DELAY(200);
        CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_RESET);

        for (i = 0; i < RE_TIMEOUT; i++) {
                DELAY(10);
                if (!(CSR_READ_1(sc, RE_COMMAND) & RE_CMD_RESET))
                        break;
        }

        if (i == RE_TIMEOUT)
                device_printf(sc->dev,"reset never completed!\n");

        return;
}

static u_int8_t
re_set_wol_linkspeed(struct re_softc *sc)
{
        u_int8_t wol_link_speed = 0xff;

        if (HW_SUPP_SERDES_PHY(sc))
                goto exit;

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(sc))
                goto exit;
#endif //ENABLE_FIBER_SUPPORT

        re_mdio_write(sc, 0x1F, 0x0000);

        wol_link_speed = RE_WOL_LINK_SPEED_100M_FIRST;
        if (!sc->re_dash) {
                u_int16_t anlpar;

                anlpar = sc->phy_reg_anlpar;

                if ((anlpar & ANLPAR_10_FD) || (anlpar & ANLPAR_10)) {
                        wol_link_speed = RE_WOL_LINK_SPEED_10M_FIRST;
                } else {
                        wol_link_speed = RE_WOL_LINK_SPEED_100M_FIRST;
                }
        }

        switch(sc->re_device_id) {
        case RT_DEVICEID_8126:
                re_clear_eth_ocp_phy_bit(sc, 0xA5D4, RTK_ADVERTISE_5000FULL);
        /*	FALLTHROUGH */
        case RT_DEVICEID_8125:
        case RT_DEVICEID_3000:
        case RT_DEVICEID_8162:
                re_clear_eth_ocp_phy_bit(sc, 0xA5D4, RTK_ADVERTISE_2500FULL);
        /*	FALLTHROUGH */
        case RT_DEVICEID_8169:
        case RT_DEVICEID_8169SC:
        case RT_DEVICEID_8168:
        case RT_DEVICEID_8161:
                re_mdio_write(sc, MII_100T2CR, re_mdio_read(sc,MII_100T2CR) & ~(GTCR_ADV_1000TFDX|GTCR_ADV_1000THDX));
        /*	FALLTHROUGH */
        default:
                if (wol_link_speed == RE_WOL_LINK_SPEED_10M_FIRST)
                        re_mdio_write(sc, MII_ANAR, re_mdio_read(sc,MII_ANAR) & ~(ANAR_TX_FD | ANAR_TX));
                break;
        }

        switch(sc->re_device_id) {
        case RT_DEVICEID_8126:
        case RT_DEVICEID_8125:
        case RT_DEVICEID_3000:
        case RT_DEVICEID_8162:
        case RT_DEVICEID_8169:
        case RT_DEVICEID_8169SC:
        case RT_DEVICEID_8168:
        case RT_DEVICEID_8161:
                re_mdio_write(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN | BMCR_STARTNEG);
                break;
        default:
                if (sc->re_type == MACFG_36)
                        re_mdio_write(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN | BMCR_STARTNEG);
                else
                        re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
                break;
        }

exit:
        return wol_link_speed;
}

static void
re_setwol(struct re_softc *sc)
{
        struct ifnet            *ifp;
        int                     pmc;
        uint16_t                pmstat;
        uint8_t                 v;

        RE_LOCK_ASSERT(sc);

        ifp = RE_GET_IFNET(sc);

        if ((ifp->if_capenable & IFCAP_WOL) == 0) {
                re_phy_power_down(sc->dev);
                return;
        }

        if (pci_find_cap(sc->dev, PCIY_PMG, &pmc) != 0)
                return;

        /* Enable config register write. */
        re_enable_cfg9346_write(sc);

        if (ifp->if_capenable & IFCAP_WOL_MAGIC)
                re_enable_magic_packet(sc);
        else
                re_disable_magic_packet(sc);

        v = CSR_READ_1(sc, RE_CFG5);
        v &= ~(RL_CFG5_WOL_BCAST | RL_CFG5_WOL_MCAST | RL_CFG5_WOL_UCAST |
               RL_CFG5_WOL_LANWAKE);

        if ((ifp->if_capenable & IFCAP_WOL_UCAST) != 0)
                v |= RL_CFG5_WOL_UCAST;
        if ((ifp->if_capenable & IFCAP_WOL_MCAST) != 0)
                v |= RL_CFG5_WOL_MCAST | RL_CFG5_WOL_BCAST;
        if ((ifp->if_capenable & IFCAP_WOL) != 0)
                v |= RL_CFG5_WOL_LANWAKE;
        CSR_WRITE_1(sc, RE_CFG5, v);

        /* Config register write done. */
        re_disable_cfg9346_write(sc);

        /*
         * It seems that hardware resets its link speed to 100Mbps in
         * power down mode so switching to 100Mbps in driver is not
         * needed.
         */

        /* Request PME if WOL is requested. */
        pmstat = pci_read_config(sc->dev, pmc + PCIR_POWER_STATUS, 2);
        pmstat &= ~(PCIM_PSTAT_PME | PCIM_PSTAT_PMEENABLE);
        if ((ifp->if_capenable & IFCAP_WOL) != 0)
                pmstat |= PCIM_PSTAT_PME | PCIM_PSTAT_PMEENABLE;
        pci_write_config(sc->dev, pmc + PCIR_POWER_STATUS, pmstat, 2);

        /* Put controller into sleep mode. */
        if ((ifp->if_capenable & IFCAP_WOL) != 0) {
                uint8_t wol_link_speed;
                re_set_rx_packet_filter_in_sleep_state(sc);
                wol_link_speed = re_set_wol_linkspeed(sc);
                if (sc->re_type == MACFG_21 || sc->re_type == MACFG_22)
                        CSR_WRITE_1(sc, RE_COMMAND, RE_CMD_RX_ENB);

                if (sc->RequiredPfmPatch)
                        re_set_pfm_patch(sc,
                                         (wol_link_speed == RE_WOL_LINK_SPEED_10M_FIRST) ?
                                         1 : 0);
        }
}

static void
re_clrwol(struct re_softc *sc)
{
        int                     pmc;
        uint16_t                pmstat;
        uint8_t                 v;

        RE_LOCK_ASSERT(sc);

        if (pci_find_cap(sc->dev, PCIY_PMG, &pmc) != 0)
                return;

        /* Disable PME and clear PME status. */
        pmstat = pci_read_config(sc->dev, pmc + PCIR_POWER_STATUS, 2);
        pmstat &= ~PCIM_PSTAT_PMEENABLE;
        pci_write_config(sc->dev, pmc + PCIR_POWER_STATUS, pmstat, 2);

        /* Enable config register write. */
        re_enable_cfg9346_write(sc);

        v = CSR_READ_1(sc, RE_CFG3);
        v &= ~(RL_CFG3_WOL_LINK);
        CSR_WRITE_1(sc, RE_CFG3, v);

        re_disable_magic_packet(sc);

        v = CSR_READ_1(sc, RE_CFG5);
        v &= ~(RL_CFG5_WOL_BCAST | RL_CFG5_WOL_MCAST | RL_CFG5_WOL_UCAST);
        v &= ~RL_CFG5_WOL_LANWAKE;
        CSR_WRITE_1(sc, RE_CFG5, v);

        /* Config register write done. */
        re_disable_cfg9346_write(sc);
}

/*
 * Stop the adapter and free any mbufs allocated to the
 * RX and TX lists.
 */
static void re_stop(struct re_softc *sc)  	/* Stop Driver */
{
        struct ifnet		*ifp;

        /*	RE_LOCK_ASSERT(sc);*/

        ifp = RE_GET_IFNET(sc);
#if OS_VER < VERSION(9,0)
        ifp->if_timer = 0;
#endif

        re_stop_timer(sc);

        /*
         * Disable accepting frames to put RX MAC into idle state.
         * Otherwise it's possible to get frames while stop command
         * execution is in progress and controller can DMA the frame
         * to already freed RX buffer during that period.
         */
        re_clear_all_rx_packet_filter(sc);

        switch (sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                CSR_WRITE_4(sc, RE_IMR0_8125, 0x00000000);
                CSR_WRITE_4(sc, RE_ISR0_8125, 0xffffffff);
                break;
        default:
                CSR_WRITE_2(sc, RE_IMR, 0x0000);
                CSR_WRITE_2(sc, RE_ISR, 0xffff);
                break;
        }

        switch (sc->re_type) {
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
                re_eri_write(sc, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                re_eri_write(sc, 0x1dc, 4, 0x0000002d, ERIAR_ExGMAC);
                break;
        case MACFG_38:
                re_eri_write(sc, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                re_eri_write(sc, 0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
                break;
        }

        switch (sc->re_type) {
        case MACFG_74:
        case MACFG_75:
                re_set_mac_ocp_bit(sc, 0xD438, BIT_3);
                re_set_mac_ocp_bit(sc, 0xD438, BIT_2);
                re_clear_mac_ocp_bit(sc, 0xDE28, (BIT_1 | BIT_0));
                re_set_mac_ocp_bit(sc, 0xD438, (BIT_1 | BIT_0));
                break;
        }

        re_reset(sc);

        /*
         * Free the TX list buffers.
         */
        while (sc->re_desc.tx_last_index!=sc->re_desc.tx_cur_index) {
                u_int32_t entry = sc->re_desc.tx_last_index % RE_TX_BUF_NUM;
                if (sc->re_desc.re_tx_mtag) {
                        bus_dmamap_sync(sc->re_desc.re_tx_mtag,
                                        sc->re_desc.re_tx_dmamap[entry],
                                        BUS_DMASYNC_POSTWRITE);
                        bus_dmamap_unload(sc->re_desc.re_tx_mtag,
                                          sc->re_desc.re_tx_dmamap[entry]);
                }

                if (sc->re_desc.tx_buf[entry]!=NULL) {
                        m_freem(sc->re_desc.tx_buf[entry]);
                        sc->re_desc.tx_buf[entry] = NULL;
                }
                sc->re_desc.tx_last_index++;
        }

        ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

        return;
}

/*
 * Main transmit routine.
 */
static void re_start(struct ifnet *ifp)  	/* Transmit Packet*/
{
        struct re_softc		*sc;

        sc = ifp->if_softc;	/* Paste to ifp in function re_attach(dev) */
        RE_LOCK(sc);
        re_start_locked(ifp);
        RE_UNLOCK(sc);
}

static int
re_get_l4hdr_offset(struct mbuf *mb)
{
        struct ether_vlan_header *eh;
        //struct tcphdr *th;
        struct ip *ip;
        int ip_hlen;
        struct ip6_hdr *ip6;
        uint16_t eth_type;
        int eth_hdr_len;

        eh = mtod(mb, struct ether_vlan_header *);
        if (mb->m_len < ETHER_HDR_LEN)
                return (0);
        if (eh->evl_encap_proto == htons(ETHERTYPE_VLAN)) {
                eth_type = ntohs(eh->evl_proto);
                eth_hdr_len = ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN;
        } else {
                eth_type = ntohs(eh->evl_encap_proto);
                eth_hdr_len = ETHER_HDR_LEN;
        }
        if (mb->m_len < eth_hdr_len)
                return (0);
        switch (eth_type) {
        case ETHERTYPE_IP:
                ip = (struct ip *)(mb->m_data + eth_hdr_len);
                if (mb->m_len < eth_hdr_len + sizeof(*ip))
                        return (0);
                if (ip->ip_p != IPPROTO_TCP)
                        return (0);
                ip_hlen = ip->ip_hl << 2;
                eth_hdr_len += ip_hlen;
                break;
        case ETHERTYPE_IPV6:
                ip6 = (struct ip6_hdr *)(mb->m_data + eth_hdr_len);
                if (mb->m_len < eth_hdr_len + sizeof(*ip6))
                        return (0);
                if (ip6->ip6_nxt != IPPROTO_TCP)
                        return (0);
                eth_hdr_len += sizeof(*ip6);
                break;
        default:
                return (0);
        }
        /*
        if (mb->m_len < eth_hdr_len + sizeof(*th))
        return (0);
        th = (struct tcphdr *)(mb->m_data + eth_hdr_len);
        tcp_hlen = th->th_off << 2;
        eth_hdr_len += tcp_hlen;
        if (mb->m_len < eth_hdr_len)
        return (0);
        */
        return (eth_hdr_len);
}

static void re_start_locked(struct ifnet *ifp)
{
        bus_dma_segment_t	segs[RE_NTXSEGS];
        bus_dmamap_t		map;
        struct re_softc		*sc;
        struct mbuf		*m_head;
        struct mbuf		*m_new;
        uint32_t tx_cur_index;
        uint32_t entry;
        uint32_t first_entry;
        int		queued;
        int		nsegs;
        int		error;
        int		i;

        sc = ifp->if_softc;	/* Paste to ifp in function re_attach(dev) */

        /*	RE_LOCK_ASSERT(sc);*/

        if ((sc->driver_detach == 1) || (sc->rx_fifo_overflow != 0))
                return;

        tx_cur_index = sc->re_desc.tx_cur_index;
        for (queued = 0; !IFQ_DRV_IS_EMPTY(&ifp->if_snd);) {
                int fs = 1, ls = 0;
                uint32_t  opts1;
                uint32_t  opts2;
                IFQ_DRV_DEQUEUE(&ifp->if_snd, m_head);	/* Remove(get) data from system transmit queue */
                if (m_head == NULL) {
                        break;
                }

                if ((sc->re_type == MACFG_80 || sc->re_type == MACFG_81 ||
                    sc->re_type == MACFG_82 || sc->re_type == MACFG_83) &&
                    sc->re_device_id != RT_DEVICEID_3000) {
                        if (re_8125_pad(sc, m_head) != 0) {
                                IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
                                ifp->if_drv_flags |= IFF_DRV_OACTIVE;
                                break;
                        }
                }

                entry = tx_cur_index % RE_TX_BUF_NUM;
                if (sc->re_coalesce_tx_pkt) {
                        if (re_encap(sc, &m_head)) {
                                IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
                                ifp->if_drv_flags |= IFF_DRV_OACTIVE;
                                break;
                        }
                }

                error = bus_dmamap_load_mbuf_sg(sc->re_desc.re_tx_mtag, sc->re_desc.re_tx_dmamap[entry],
                                                m_head, segs, &nsegs, BUS_DMA_NOWAIT);
                if (error == EFBIG) {
                        m_new = m_collapse(m_head, M_NOWAIT, RE_NTXSEGS);
                        if (m_new == NULL) {
                                m_freem(m_head);
                                m_head = NULL;
                                //return (ENOBUFS);
                                break;
                        }
                        m_head = m_new;
                        error = bus_dmamap_load_mbuf_sg(sc->re_desc.re_tx_mtag,
                                                        sc->re_desc.re_tx_dmamap[entry], m_head, segs, &nsegs, BUS_DMA_NOWAIT);
                        if (error != 0) {
                                m_freem(m_head);
                                m_head = NULL;
                                //return (error);
                                break;
                        }
                } else if (error != 0) {
                        //return (error);
                        IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
                        ifp->if_drv_flags |= IFF_DRV_OACTIVE;
                        break;
                }
                if (nsegs == 0) {
                        m_freem(m_head);
                        m_head = NULL;
                        //return (EIO);
                        break;
                }

                /* Check for number of available descriptors. */
                if (CountFreeTxDescNum(&sc->re_desc) < nsegs) {	/* No enough descriptor */
                        bus_dmamap_unload(sc->re_desc.re_tx_mtag, sc->re_desc.re_tx_dmamap[entry]);
                        IFQ_DRV_PREPEND(&ifp->if_snd, m_head);
                        ifp->if_drv_flags |= IFF_DRV_OACTIVE;
                        break;
                }

                bus_dmamap_sync(sc->re_desc.re_tx_mtag, sc->re_desc.re_tx_dmamap[entry],
                                BUS_DMASYNC_PREWRITE);

                first_entry = entry;

                if (ifp->if_bpf) {		/* If there's a BPF listener, bounce a copy of this frame to him. */
                        //printf("If there's a BPF listener, bounce a copy of this frame to him. \n");

                        /*#if OS_VER < VERSION(5, 1)*/
#if OS_VER < VERSION(4,9)
                        bpf_mtap(ifp, m_head);
#else
                        bpf_mtap(ifp->if_bpf, m_head);
#endif
                }

                opts1 = opts2 = 0;
                //hw tso
                if ((m_head->m_pkthdr.csum_flags & CSUM_TSO) != 0) {
                        if ((sc->re_if_flags & RL_FLAG_DESCV2) == 0) {
                                opts2 |= RL_TDESC_CMD_LGSEND |
                                         ((uint32_t)m_head->m_pkthdr.tso_segsz <<
                                          RL_TDESC_CMD_MSSVAL_SHIFT);
                        } else {
                                /*
                                 * RTL8168C/RTL816CP/RTL8111C/RTL8111CP
                                 */
                                const uint16_t l4hoffset =
                                        re_get_l4hdr_offset(m_head);

                                if (re_get_eth_type(m_head) == ETHERTYPE_IPV6)
                                        opts1 |= RL_TDESC_CMD_GTSENDV6;
                                else
                                        opts1 |= RL_TDESC_CMD_GTSENDV4;
                                opts1 |= (l4hoffset << RL_TDESC_CMD_GTSEND_TCPHO_SHIFT);
                                opts2 |= ((uint32_t)m_head->m_pkthdr.tso_segsz <<
                                          RL_TDESC_CMD_MSSVALV2_SHIFT);
                        }
                        goto process_vlan;
                }
                //hw checksum
                if ((m_head->m_pkthdr.csum_flags & RE_CSUM_FEATURES) != 0) {
                        if ((sc->re_if_flags & RL_FLAG_DESCV2) == 0) {
                                opts1 |= RL_IPV4CS1;
                                if (m_head->m_pkthdr.csum_flags & CSUM_TCP)
                                        opts1 |= RL_TCPCS1;
                                if (m_head->m_pkthdr.csum_flags & CSUM_UDP)
                                        opts1 |= RL_UDPCS1;
                        } else {
                                /*
                                 * RTL8168C/RTL816CP/RTL8111C/RTL8111CP
                                 */
                                if (re_get_eth_type(m_head) == ETHERTYPE_IP)
                                        opts2 |= RL_IPV4CS;
                                if (m_head->m_pkthdr.csum_flags &
                                    (CSUM_TCP | CSUM_TCP_IPV6))
                                        opts2 |= RL_TCPCS;
                                else if (m_head->m_pkthdr.csum_flags &
                                         (CSUM_UDP | CSUM_UDP_IPV6))
                                        opts2 |= RL_UDPCS;
                                if (m_head->m_pkthdr.csum_flags &
                                    (CSUM_TCP_IPV6 | CSUM_UDP_IPV6)) {
                                        const uint16_t l4hoffset =
                                                re_get_l4hdr_offset(m_head);

                                        opts2 |= RL_CS_V6F |
                                                 (l4hoffset << RL_TDESC_CMD_CSUM_TCPHO_SHIFT);
                                }
                        }
                        goto process_vlan;
                }
process_vlan:
                //vlan
                if (m_head->m_flags & M_VLANTAG)
                        opts2 |= bswap16(m_head->m_pkthdr.ether_vtag) | RL_TDESC_VLANCTL_TAG;
#ifdef _DEBUG_
                printf("PktLen=%d \n", m_head->m_pkthdr.len);
#endif
                for (i = 0; i < nsegs; i++) {
                        if (segs[i].ds_len == 0)
                                continue;

                        if (i == (nsegs - 1)) {
                                ls=1;

                                /*
                                * Insure that the map for this transmission
                                * is placed at the array index of the last descriptor
                                * in this chain.  (Swap last and first dmamaps.)
                                */
                                map = sc->re_desc.re_tx_dmamap[first_entry];
                                sc->re_desc.re_tx_dmamap[first_entry] = sc->re_desc.re_tx_dmamap[entry];
                                sc->re_desc.re_tx_dmamap[entry] = map;
                                sc->re_desc.tx_buf[entry] = m_head;
                        } else
                                sc->re_desc.tx_buf[entry] = NULL;

                        WritePacket(sc,&segs[i],fs,ls,opts2,opts1, entry);

                        fs=0;

                        tx_cur_index++;
                        entry = tx_cur_index % RE_TX_BUF_NUM;
                }
                sc->re_desc.tx_cur_index = tx_cur_index;
#ifdef _DEBUG_
                printf("\n");
#endif
                queued++;
        }

        sc->re_desc.tx_cur_index = tx_cur_index;

        if (queued == 0)
                return;

        re_start_tx(sc);

#if OS_VER < VERSION(9,0)
        ifp->if_timer = 5;
#endif

        return;
}

static void _re_start_tx(struct re_softc	*sc)
{
        switch (sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                CSR_WRITE_2(sc, RE_TPPOLL_8125, RE_NPQ_8125);
                break;
        default:
                CSR_WRITE_1(sc, RE_TPPOLL, RE_NPQ);
                break;
        }
}

static void re_start_tx(struct re_softc	*sc)
{
        bus_dmamap_sync(sc->re_desc.tx_desc_tag,
                        sc->re_desc.tx_desc_dmamap,
                        BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

        _re_start_tx(sc);
}

/*
 * Encapsulate an mbuf chain in a descriptor by coupling the mbuf data
 * pointers to the fragment pointers.
 */
static int re_encap(struct re_softc *sc,struct mbuf **m_head)
{
        struct mbuf		*m_new = NULL;

        m_new = m_defrag(*m_head, M_DONTWAIT);

        if (m_new == NULL) {
                printf("re%d: no memory for tx list", sc->re_unit);
                return (1);
        }

        /* Pad frames to at least 60 bytes. */
        if (m_new->m_pkthdr.len < RE_MIN_FRAMELEN) {	/* Case length < 60 bytes */
                /*
                 * Make security concious people happy: zero out the
                 * bytes in the pad area, since we don't know what
                 * this mbuf cluster buffer's previous user might
                 * have left in it.
                 */
                bzero(mtod(m_new, char *) + m_new->m_pkthdr.len,
                      RE_MIN_FRAMELEN - m_new->m_pkthdr.len);
                m_new->m_pkthdr.len = RE_MIN_FRAMELEN;
                m_new->m_len = m_new->m_pkthdr.len;
        }

        *m_head = m_new;

        return(0);
}

static void WritePacket(struct re_softc	*sc, bus_dma_segment_t *segs, int fs, int ls, uint32_t opts2, uint32_t opts1, uint32_t entry)
{
        union TxDesc *txptr;
        uint32_t status;

        txptr = &(sc->re_desc.tx_desc[entry]);

        status = RL_TDESC_CMD_OWN | opts1 | segs->ds_len;

        if (fs)
                status |= RL_TDESC_CMD_SOF;
        if (ls)
                status |= RL_TDESC_CMD_EOF;
        if (entry == (RE_TX_BUF_NUM - 1))
                status |= RL_TDESC_CMD_EOR;

        txptr->so1.TxBuff = htole64(segs->ds_addr);

        txptr->ul[1] = htole32(opts2);
        /* make sure opts2 is set before opts1 */
        wmb();
        txptr->ul[0] = htole32(status);
}

static uint32_t CountFreeTxDescNum(struct re_descriptor *desc)
{
        uint32_t ret=desc->tx_last_index + RE_TX_BUF_NUM - desc->tx_cur_index;

        return ret;
}

/*
static int CountMbufNum(struct mbuf *m_head)
{
        int ret=0;
        struct mbuf *ptr = m_head;

        while (ptr!=NULL) {
                if (ptr->m_len >0)
                        ret++;
                ptr=ptr->m_next;
        }

        return ret;
}
*/

#ifdef RE_FIXUP_RX
static __inline void re_fixup_rx(struct mbuf *m)
{
        int                     i;
        uint16_t                *src, *dst;

        src = mtod(m, uint16_t *);
        dst = src - (RE_ETHER_ALIGN - ETHER_ALIGN) / sizeof *src;

        for (i = 0; i < (m->m_len / sizeof(uint16_t) + 1); i++)
                *dst++ = *src++;

        m->m_data -= RE_ETHER_ALIGN - ETHER_ALIGN;
}
#endif

/*
 * A frame was downloaded to the chip. It's safe for us to clean up
 * the list buffers.
 */
static void re_txeof(struct re_softc *sc)  	/* Transmit OK/ERR handler */
{
        union TxDesc *txptr;
        struct ifnet		*ifp;
        u_int32_t           txstat;
        u_int32_t           entry;
        u_int32_t           tx_cur_index;
        u_int32_t           tx_last_index;

        /*	printf("X");*/

        ifp = RE_GET_IFNET(sc);

#if OS_VER < VERSION(9,0)
        /* Clear the timeout timer. */
        ifp->if_timer = 0;
#endif
        tx_cur_index = sc->re_desc.tx_cur_index;
        tx_last_index = sc->re_desc.tx_last_index;
        /* No packet to complete. */
        if (tx_cur_index == tx_last_index)
                return;

        bus_dmamap_sync(sc->re_desc.tx_desc_tag,
                        sc->re_desc.tx_desc_dmamap,
                        BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);

        while (tx_cur_index != tx_last_index) {
                entry = tx_last_index % RE_TX_BUF_NUM;
                txptr=&(sc->re_desc.tx_desc[entry]);
                txstat = le32toh(txptr->ul[0]);
                if (txstat & RL_TDESC_STAT_OWN)
                        break;
#ifdef _DEBUG_
                printf("**** Tx OK  ****\n");
#endif
                if (sc->re_desc.tx_buf[entry]!=NULL) {
                        bus_dmamap_sync(sc->re_desc.re_tx_mtag,
                                        sc->re_desc.re_tx_dmamap[entry],
                                        BUS_DMASYNC_POSTWRITE);
                        bus_dmamap_unload(sc->re_desc.re_tx_mtag,
                                          sc->re_desc.re_tx_dmamap[entry]);
                        /* Free Current MBuf in a Mbuf list*/
                        m_freem(sc->re_desc.tx_buf[entry]);
                        sc->re_desc.tx_buf[entry] = NULL;
                }

                tx_last_index++;
        }

        if (sc->re_desc.tx_last_index != tx_last_index) {
                sc->re_desc.tx_last_index = tx_last_index;
                ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
        }

        /* prevent tx stop. */
        if (tx_cur_index != tx_last_index)
                _re_start_tx(sc);

        return;
}

#if defined(INET) || defined(INET6)
static int
re_lro_rx(struct re_softc *sc, struct mbuf *m)
{
        struct lro_ctrl *lro;

        lro = &sc->re_lro;

        if (lro->lro_mbuf_max != 0) {
                tcp_lro_queue_mbuf(lro, m);
                return (0);
        }

        return (tcp_lro_rx(lro, m, 0));
}
#endif

static void
re_rxq_input(struct re_softc *sc, struct mbuf *m, bool lro_able)
{
        struct ifnet		*ifp;

        ifp = RE_GET_IFNET(sc);

#if defined(INET) || defined(INET6)
        if ((ifp->if_capenable & IFCAP_LRO) && lro_able) {
                if (re_lro_rx(sc, m) == 0)
                        return;
        }
#endif

        /*#if OS_VER < VERSION(5, 1)*/
#if OS_VER < VERSION(4,9)
        /* Remove header from mbuf and pass it on. */
        m_adj(m, sizeof(struct ether_header));
        ether_input(ifp, eh, m);
#else
        (*ifp->if_input)(ifp, m);
#endif
}

static void
re_drain_soft_lro(struct re_softc *sc)
{
#if defined(INET) || defined(INET6)
#if OS_VER >= VERSION(11,0)
        tcp_lro_flush_all(&sc->re_lro);
#else
        struct lro_entry *queued;

        while ((!SLIST_EMPTY(&lro->lro_active))) {
                queued = SLIST_FIRST(&lro->lro_active);
                SLIST_REMOVE_HEAD(&lro->lro_active, next);
                tcp_lro_flush(lro, queued);
        }
#endif //OS_VER >= VERSION(11,0)
#endif //defined(INET) || defined(INET6)
}

/*
 * A frame has been uploaded: pass the resulting mbuf chain up to
 * the higher level protocols.
 *
 * You know there's something wrong with a PCI bus-master chip design
 * when you have to use m_devget().
 *
 * The receive operation is badly documented in the datasheet, so I'll
 * attempt to document it here. The driver provides a buffer area and
 * places its base address in the RX buffer start address register.
 * The chip then begins copying frames into the RX buffer. Each frame
 * is preceeded by a 32-bit RX status word which specifies the length
 * of the frame and certain other status bits. Each frame (starting with
 * the status word) is also 32-bit aligned. The frame length is in the
 * first 16 bits of the status word; the lower 15 bits correspond with
 * the 'rx status register' mentioned in the datasheet.
 *
 * Note: to make the Alpha happy, the frame payload needs to be aligned
 * on a 32-bit boundary. To achieve this, we cheat a bit by copying from
 * the ring buffer starting at an address two bytes before the actual
 * data location. We can then shave off the first two bytes using m_adj().
 * The reason we do this is because m_devget() doesn't let us specify an
 * offset into the mbuf storage space, so we have to artificially create
 * one. The ring is allocated in such a way that there are a few unused
 * bytes of space preceecing it so that it will be safe for us to do the
 * 2-byte backstep even if reading from the ring at offset 0.
 */
static int re_rxeof(struct re_softc *sc)	/* Receive Data OK/ERR handler */
{
        struct mbuf		*m;
        struct ifnet		*ifp;
        union RxDesc *rxptr;
        struct mbuf *buf;
        int size;
        int maxpkt = RE_RX_BUDGET;
        u_int32_t entry;
        u_int32_t pkt_size;
        u_int32_t rx_cur_index;
        u_int32_t opts2,opts1,status;
        bool lro_able;

        /* RE_LOCK_ASSERT(sc);*/

        ifp = RE_GET_IFNET(sc);

        bus_dmamap_sync(sc->re_desc.rx_desc_tag,
                        sc->re_desc.rx_desc_dmamap,
                        BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);

        rx_cur_index = sc->re_desc.rx_cur_index;
        while (maxpkt > 0) {
                entry = rx_cur_index % RE_RX_BUF_NUM;
                rxptr=&(sc->re_desc.rx_desc[entry]);
                opts1 = le32toh(rxptr->ul[0]);
                lro_able = false;

                /* Check Receive OK */
                if (opts1&RL_RDESC_STAT_OWN)
                        break;

                /* Check if this packet is received correctly*/
                if (opts1&RL_RDESC_RES)	/*Check RES bit*/
                        goto drop_packet;

                pkt_size = (opts1&RL_RDESC_STAT_GFRAGLEN)-ETHER_CRC_LEN;
                if (pkt_size > sc->re_rx_mbuf_sz)
                        goto drop_packet;

                /* Wait other fields is dmaed */
                rmb();

                opts2 = le32toh(rxptr->ul[1]);

                //buf = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR); /* Alloc a new mbuf */

                if (sc->re_rx_mbuf_sz <= MCLBYTES)
                        size = MCLBYTES;
                else if (sc->re_rx_mbuf_sz <= MJUMPAGESIZE)
                        size = MJUMPAGESIZE;
                else
                        size = MJUM9BYTES;

                buf = m_getjcl(M_DONTWAIT, MT_DATA, M_PKTHDR, size);
                if (buf==NULL)
                        goto drop_packet;

                buf->m_len = buf->m_pkthdr.len = size;
#ifdef RE_FIXUP_RX
                /*
                 * This is part of an evil trick to deal with non-x86 platforms.
                 * The RealTek chip requires RX buffers to be aligned on 64-bit
                 * boundaries, but that will hose non-x86 machines. To get around
                 * this, we leave some empty space at the start of each buffer
                 * and for non-x86 hosts, we copy the buffer back six bytes
                 * to achieve word alignment. This is slightly more efficient
                 * than allocating a new buffer, copying the contents, and
                 * discarding the old buffer.
                 */
                m_adj(buf, RE_ETHER_ALIGN);
#endif

                bus_dmamap_sync(sc->re_desc.re_rx_mtag,
                                sc->re_desc.re_rx_dmamap[entry],
                                BUS_DMASYNC_POSTREAD);
                bus_dmamap_unload(sc->re_desc.re_rx_mtag,
                                  sc->re_desc.re_rx_dmamap[entry]);

                m = sc->re_desc.rx_buf[entry];
                sc->re_desc.rx_buf[entry] = buf;
                m->m_pkthdr.len = m->m_len = pkt_size;
                m->m_pkthdr.rcvif = ifp;

#ifdef RE_FIXUP_RX
                re_fixup_rx(m);
#endif

                //vlan
                if (opts2 & RL_RDESC_VLANCTL_TAG) {
                        m->m_pkthdr.ether_vtag =
                                bswap16((opts2 & RL_RDESC_VLANCTL_DATA));
                        m->m_flags |= M_VLANTAG;
                }
                if (ifp->if_capenable & IFCAP_RXCSUM) {
                        if ((sc->re_if_flags & RL_FLAG_DESCV2) == 0) {
                                u_int32_t proto = opts1 & RL_ProtoMASK;

                                if (proto != 0)
                                        m->m_pkthdr.csum_flags |=  CSUM_IP_CHECKED;
                                if (!(opts1 & RL_IPF))
                                        m->m_pkthdr.csum_flags |= CSUM_IP_VALID;
                                if (((proto == RL_ProtoTCP) && !(opts1 & RL_TCPF)) ||
                                    ((proto == RL_ProtoUDP) && !(opts1 & RL_UDPF))) {
                                        m->m_pkthdr.csum_flags |= CSUM_DATA_VALID|CSUM_PSEUDO_HDR;
                                        m->m_pkthdr.csum_data = 0xffff;
                                        if (proto == RL_ProtoTCP)
                                                lro_able = true;
                                }
                        } else {
                                /*
                                 * RTL8168C/RTL816CP/RTL8111C/RTL8111CP
                                 */
                                if (opts2 & RL_V4F)
                                        m->m_pkthdr.csum_flags |= CSUM_IP_CHECKED;
                                if ((opts2 & RL_V4F) && !(opts1 & RL_IPF))
                                        m->m_pkthdr.csum_flags |= CSUM_IP_VALID;
                                if (((opts1 & RL_TCPT) && !(opts1 & RL_TCPF)) ||
                                    ((opts1 & RL_UDPT) && !(opts1 & RL_UDPF))) {
                                        m->m_pkthdr.csum_flags |= CSUM_DATA_VALID|CSUM_PSEUDO_HDR;
                                        m->m_pkthdr.csum_data = 0xffff;
                                        if (opts1 & RL_TCPT)
                                                lro_able = true;
                                }
                        }
                }

#if OS_VER < VERSION(11,0)
                ifp->if_ipackets++;
#else
                if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
#endif
#ifdef _DEBUG_
                printf("Rcv Packet, Len=%d \n", m->m_len);
#endif

                RE_UNLOCK(sc);

                re_rxq_input(sc, m, lro_able);

                RE_LOCK(sc);

                /* Load the map for rx buffer. */
                bus_dmamap_load(sc->re_desc.re_rx_mtag,
                                sc->re_desc.re_rx_dmamap[entry],
                                sc->re_desc.rx_buf[entry]->m_data,
                                sc->re_rx_desc_buf_sz,
                                re_rx_dma_map_buf, rxptr,
                                0);
                bus_dmamap_sync(sc->re_desc.re_rx_mtag,
                                sc->re_desc.re_rx_dmamap[entry],
                                BUS_DMASYNC_PREREAD);
update_desc:
                status = RL_RDESC_CMD_OWN | sc->re_rx_desc_buf_sz;
                if (entry == (RE_RX_BUF_NUM - 1))
                        status |= RL_RDESC_CMD_EOR;

                rxptr->ul[1]=0;
                /* make sure desc is all set before releasing it */
                wmb();
                rxptr->ul[0] = htole32(status);

                rx_cur_index++;

                maxpkt--;

                continue;
drop_packet:
#if OS_VER < VERSION(11,0)
                ifp->if_ierrors++;
#else
                if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
#endif
                goto update_desc;
        }

        if (sc->re_desc.rx_cur_index != rx_cur_index) {
                if (ifp->if_capenable & IFCAP_LRO) {
                        RE_UNLOCK(sc);
                        re_drain_soft_lro(sc);
                        RE_LOCK(sc);
                }
                sc->re_desc.rx_cur_index = rx_cur_index;
                bus_dmamap_sync(sc->re_desc.rx_desc_tag,
                                sc->re_desc.rx_desc_dmamap,
                                BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);
        }

        return (RE_RX_BUDGET - maxpkt);
}

#if OS_VER < VERSION(7,0)
static void re_intr(void *arg)  	/* Interrupt Handler */
#else
static int re_intr(void *arg)  	/* Interrupt Handler */
#endif //OS_VER < VERSION(7,0)
{
        struct re_softc		*sc;

        sc = arg;

        if ((sc->re_if_flags & (RL_FLAG_MSI | RL_FLAG_MSIX)) == 0) {
                if ((CSR_READ_2(sc, RE_ISR) & RE_INTRS) == 0) {
#if OS_VER < VERSION(7,0)
                        return;
#else
                        return (FILTER_STRAY);
#endif
                }
        }

        /* Disable interrupts. */
        CSR_WRITE_2(sc, RE_IMR, 0x0000);

#if OS_VER < VERSION(7,0)
        re_int_task(arg, 0);
#else //OS_VER < VERSION(7,0)
#if OS_VER < VERSION(11,0)
        taskqueue_enqueue_fast(sc->re_tq, &sc->re_inttask);
#else ////OS_VER < VERSION(11,0)
        taskqueue_enqueue(sc->re_tq, &sc->re_inttask);
#endif //OS_VER < VERSION(11,0)
        return (FILTER_HANDLED);
#endif //OS_VER < VERSION(7,0)
}

#if OS_VER < VERSION(7,0)
static void re_intr_8125(void *arg)  	/* Interrupt Handler */
#else
static int re_intr_8125(void *arg)  	/* Interrupt Handler */
#endif //OS_VER < VERSION(7,0)
{
        struct re_softc		*sc;

        sc = arg;

        if ((sc->re_if_flags & (RL_FLAG_MSI | RL_FLAG_MSIX)) == 0) {
                if ((CSR_READ_4(sc, RE_ISR0_8125) & RE_INTRS) == 0) {
#if OS_VER < VERSION(7,0)
                        return;
#else
                        return (FILTER_STRAY);
#endif
                }
        }

        /* Disable interrupts. */
        CSR_WRITE_4(sc, RE_IMR0_8125, 0x00000000);

#if OS_VER < VERSION(7,0)
        re_int_task_8125(arg, 0);
#else //OS_VER < VERSION(7,0)
#if OS_VER < VERSION(11,0)
        taskqueue_enqueue_fast(sc->re_tq, &sc->re_inttask);
#else ////OS_VER < VERSION(11,0)
        taskqueue_enqueue(sc->re_tq, &sc->re_inttask);
#endif //OS_VER < VERSION(11,0)
        return (FILTER_HANDLED);
#endif //OS_VER < VERSION(7,0)
}

static void re_int_task_poll(void *arg, int npending)
{
        struct re_softc		*sc;
        struct ifnet		*ifp;
        int 				done;

        sc = arg;

        RE_LOCK(sc);

        ifp = RE_GET_IFNET(sc);

        if (sc->suspended ||
            (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
                RE_UNLOCK(sc);
                return;
        }

        done = re_rxeof(sc);

        re_txeof(sc);

        if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
                re_start_locked(ifp);

        RE_UNLOCK(sc);

#if OS_VER>=VERSION(7,0)
        if (done >= RE_RX_BUDGET) {
#if OS_VER < VERSION(11,0)
                taskqueue_enqueue_fast(sc->re_tq, &sc->re_inttask_poll);
#else ////OS_VER < VERSION(11,0)
                taskqueue_enqueue(sc->re_tq, &sc->re_inttask_poll);
#endif //OS_VER < VERSION(11,0)
                return;
        }
#endif //OS_VER>=VERSION(7,0)

        /* Re-enable interrupts. */
        CSR_WRITE_2(sc, RE_IMR, RE_INTRS);
}

static void re_int_task(void *arg, int npending)
{
        struct re_softc		*sc;
        struct ifnet		*ifp;
        int 				done;
        u_int32_t			status;

        sc = arg;

        RE_LOCK(sc);

        ifp = RE_GET_IFNET(sc);

        status = CSR_READ_2(sc, RE_ISR);

        if (status) {
                CSR_WRITE_2(sc, RE_ISR, status & ~RE_ISR_FIFO_OFLOW);
        }

        if (sc->suspended ||
            (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
                RE_UNLOCK(sc);
                return;
        }

        done = re_rxeof(sc);

        if (sc->re_type == MACFG_21) {
                if (status & RE_ISR_FIFO_OFLOW) {
                        sc->rx_fifo_overflow = 1;
                        CSR_WRITE_2(sc, RE_IntrMitigate, 0x0000);
                        CSR_WRITE_4(sc, RE_TIMERCNT, 0x4000);
                        CSR_WRITE_4(sc, RE_TIMERINT, 0x4000);
                } else {
                        sc->rx_fifo_overflow = 0;
                        CSR_WRITE_4(sc,RE_CPlusCmd, 0x51512082);
                }

                if (status & RE_ISR_PCS_TIMEOUT) {
                        if ((status & RE_ISR_FIFO_OFLOW) &&
                            (!(status & (RE_ISR_RX_OK | RE_ISR_TX_OK | RE_ISR_RX_OVERRUN)))) {
                                re_reset(sc);
                                re_init_locked(sc);
                                sc->rx_fifo_overflow = 0;
                                CSR_WRITE_2(sc, RE_ISR, RE_ISR_FIFO_OFLOW);
                        }
                }
        }

        re_txeof(sc);

        if (status & RE_ISR_SYSTEM_ERR) {
                re_reset(sc);
                re_init_locked(sc);
        }

        if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
                re_start_locked(ifp);

        RE_UNLOCK(sc);

#if OS_VER>=VERSION(7,0)
        if (done >= RE_RX_BUDGET) {
#if OS_VER < VERSION(11,0)
                taskqueue_enqueue_fast(sc->re_tq, &sc->re_inttask_poll);
#else ////OS_VER < VERSION(11,0)
                taskqueue_enqueue(sc->re_tq, &sc->re_inttask_poll);
#endif //OS_VER < VERSION(11,0)
                return;
        }
#endif //OS_VER>=VERSION(7,0)

        /* Re-enable interrupts. */
        CSR_WRITE_2(sc, RE_IMR, RE_INTRS);
}

static void re_int_task_8125_poll(void *arg, int npending)
{
        struct re_softc		*sc;
        struct ifnet		*ifp;
        int 				done;

        sc = arg;

        RE_LOCK(sc);

        ifp = RE_GET_IFNET(sc);

        if (sc->suspended ||
            (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
                RE_UNLOCK(sc);
                return;
        }

        done = re_rxeof(sc);

        re_txeof(sc);

        if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
                re_start_locked(ifp);

        RE_UNLOCK(sc);

#if OS_VER>=VERSION(7,0)
        if (done >= RE_RX_BUDGET) {
#if OS_VER < VERSION(11,0)
                taskqueue_enqueue_fast(sc->re_tq, &sc->re_inttask_poll);
#else ////OS_VER < VERSION(11,0)
                taskqueue_enqueue(sc->re_tq, &sc->re_inttask_poll);
#endif //OS_VER < VERSION(11,0)
                return;
        }
#endif //OS_VER>=VERSION(7,0)

        /* Re-enable interrupts. */
        CSR_WRITE_4(sc, RE_IMR0_8125, RE_INTRS);
}

static void re_int_task_8125(void *arg, int npending)
{
        struct re_softc		*sc;
        struct ifnet		*ifp;
        int 				done;
        u_int32_t			status;

        sc = arg;

        RE_LOCK(sc);

        ifp = RE_GET_IFNET(sc);

        status = CSR_READ_4(sc, RE_ISR0_8125);

        if (status) {
                CSR_WRITE_4(sc, RE_ISR0_8125, status & ~RE_ISR_FIFO_OFLOW);
        }

        if (sc->suspended ||
            (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
                RE_UNLOCK(sc);
                return;
        }

        done = re_rxeof(sc);

        re_txeof(sc);

        if (status & RE_ISR_SYSTEM_ERR) {
                re_reset(sc);
                re_init_locked(sc);
        }

        if (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
                re_start_locked(ifp);

        RE_UNLOCK(sc);

#if OS_VER>=VERSION(7,0)
        if (done >= RE_RX_BUDGET) {
#if OS_VER < VERSION(11,0)
                taskqueue_enqueue_fast(sc->re_tq, &sc->re_inttask_poll);
#else ////OS_VER < VERSION(11,0)
                taskqueue_enqueue(sc->re_tq, &sc->re_inttask_poll);
#endif //OS_VER < VERSION(11,0)
                return;
        }
#endif //OS_VER>=VERSION(7,0)

        /* Re-enable interrupts. */
        CSR_WRITE_4(sc, RE_IMR0_8125, RE_INTRS);
}

static void re_set_multicast_reg(struct re_softc *sc, u_int32_t mask0,
                                 u_int32_t mask4)
{
        u_int8_t  enable_cfg_reg_write = 0;

        if (sc->re_type == MACFG_5 || sc->re_type == MACFG_6)
                enable_cfg_reg_write = 1;

        if (enable_cfg_reg_write)
                re_enable_cfg9346_write(sc);
        CSR_WRITE_4(sc, RE_MAR0, mask0);
        CSR_WRITE_4(sc, RE_MAR4, mask4);
        if (enable_cfg_reg_write)
                re_disable_cfg9346_write(sc);

        return;
}

static void re_clear_all_rx_packet_filter(struct re_softc *sc)
{
        CSR_WRITE_4(sc, RE_RXCFG, CSR_READ_4(sc, RE_RXCFG) &
                    ~(RE_RXCFG_RX_ALLPHYS | RE_RXCFG_RX_INDIV | RE_RXCFG_RX_MULTI |
                      RE_RXCFG_RX_BROAD | RE_RXCFG_RX_RUNT | RE_RXCFG_RX_ERRPKT));
}

static void re_set_rx_packet_filter_in_sleep_state(struct re_softc *sc)
{
        u_int32_t		rxfilt;

        rxfilt = CSR_READ_4(sc, RE_RXCFG);

        rxfilt &= ~(RE_RXCFG_RX_ALLPHYS | RE_RXCFG_RX_INDIV | RE_RXCFG_RX_MULTI | RE_RXCFG_RX_BROAD | RE_RXCFG_RX_RUNT | RE_RXCFG_RX_ERRPKT);
        rxfilt |= (RE_RXCFG_RX_INDIV | RE_RXCFG_RX_MULTI | RE_RXCFG_RX_BROAD);

        CSR_WRITE_4(sc, RE_RXCFG, rxfilt);

        re_set_multicast_reg(sc, 0xFFFFFFFF, 0xFFFFFFFF);

        return;
}

static void re_set_rx_packet_filter(struct re_softc *sc)
{
        struct ifnet		*ifp;
        u_int32_t		rxfilt;

        ifp = RE_GET_IFNET(sc);

        rxfilt = CSR_READ_4(sc, RE_RXCFG);

        rxfilt |= RE_RXCFG_RX_INDIV;

        if (ifp->if_flags & (IFF_MULTICAST | IFF_ALLMULTI | IFF_PROMISC)) {
                rxfilt |= (RE_RXCFG_RX_ALLPHYS | RE_RXCFG_RX_MULTI);
        } else {
                rxfilt &= ~(RE_RXCFG_RX_MULTI);
        }

        if (ifp->if_flags & IFF_PROMISC) {
                rxfilt |= (RE_RXCFG_RX_ALLPHYS | RE_RXCFG_RX_RUNT | RE_RXCFG_RX_ERRPKT);
        } else {
                rxfilt &= ~(RE_RXCFG_RX_ALLPHYS | RE_RXCFG_RX_RUNT | RE_RXCFG_RX_ERRPKT);
        }

        if (ifp->if_flags & (IFF_BROADCAST | IFF_PROMISC)) {
                rxfilt |= RE_RXCFG_RX_BROAD;
        } else {
                rxfilt &= ~RE_RXCFG_RX_BROAD;
        }

        CSR_WRITE_4(sc, RE_RXCFG, rxfilt);

        re_setmulti(sc);

        return;
}

#if OS_VER >= VERSION(13,0)
static u_int
re_hash_maddr(void *arg, struct sockaddr_dl *sdl, u_int cnt)
{
	uint32_t h, *hashes = arg;

	h = ether_crc32_be(LLADDR(sdl), ETHER_ADDR_LEN) >> 26;
	if (h < 32)
		hashes[0] |= (1 << h);
	else
		hashes[1] |= (1 << (h - 32));

	return (1);
}
#endif

/*
 * Program the 64-bit multicast hash filter.
 */
static void re_setmulti(struct re_softc *sc)
{
        struct ifnet		*ifp;
        int			h = 0;
        u_int32_t		hashes[2] = { 0, 0 };
#if OS_VER < VERSION(13,0)
        struct ifmultiaddr	*ifma;
#endif
        u_int32_t		rxfilt;
        int			mcnt = 0;

        ifp = RE_GET_IFNET(sc);

        rxfilt = CSR_READ_4(sc, RE_RXCFG);

        if (ifp->if_flags & IFF_ALLMULTI || ifp->if_flags & IFF_PROMISC) {
                rxfilt |= RE_RXCFG_RX_MULTI;
                CSR_WRITE_4(sc, RE_RXCFG, rxfilt);
                re_set_multicast_reg(sc, 0xFFFFFFFF, 0xFFFFFFFF);

                return;
        }

        /* now program new ones */
#if OS_VER >= VERSION(13,0)
	mcnt = if_foreach_llmaddr(ifp, re_hash_maddr, hashes);
#else
#if OS_VER >= VERSION(12,0)
        if_maddr_rlock(ifp);
#elif OS_VER > VERSION(6,0)
        IF_ADDR_LOCK(ifp);
#endif
#if OS_VER < VERSION(4,9)
        for (ifma = ifp->if_multiaddrs.lh_first; ifma != NULL;
             ifma = ifma->ifma_link.le_next)
#elif OS_VER < VERSION(12,0)
        TAILQ_FOREACH(ifma,&ifp->if_multiaddrs,ifma_link)
#else
        CK_STAILQ_FOREACH(ifma,&ifp->if_multiaddrs,ifma_link)
#endif
        {
                if (ifma->ifma_addr->sa_family != AF_LINK)
                        continue;
                h = ether_crc32_be(LLADDR((struct sockaddr_dl *)
                                          ifma->ifma_addr), ETHER_ADDR_LEN) >> 26;
                if (h < 32)
                        hashes[0] |= (1 << h);
                else
                        hashes[1] |= (1 << (h - 32));
                mcnt++;
        }
#if OS_VER >= VERSION(12,0)
        if_maddr_runlock(ifp);
#elif OS_VER > VERSION(6,0)
        IF_ADDR_UNLOCK(ifp);
#endif
#endif

        if (mcnt) {
                if ((sc->re_if_flags & RL_FLAG_PCIE) != 0) {
                        h = bswap32(hashes[0]);
                        hashes[0] = bswap32(hashes[1]);
                        hashes[1] = h;
                }
                rxfilt |= RE_RXCFG_RX_MULTI;
        } else
                rxfilt &= ~RE_RXCFG_RX_MULTI;

        CSR_WRITE_4(sc, RE_RXCFG, rxfilt);
        re_set_multicast_reg(sc, hashes[0], hashes[1]);

        return;
}

static int re_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
        struct re_softc		*sc = ifp->if_softc;
        struct ifreq		*ifr = (struct ifreq *) data;
        /*int			s;*/
        int			error = 0;
        int mask, reinit;
        /*s = splimp();*/

        switch(command) {
        case SIOCSIFADDR:
        case SIOCGIFADDR:
                error = ether_ioctl(ifp, command, data);

                break;
        case SIOCSIFMTU:

                //printf("before mtu =%d\n",(int)ifp->if_mtu);
                if (ifr->ifr_mtu > sc->max_jumbo_frame_size) {
                        error = EINVAL;
                        break;
                }
                RE_LOCK(sc);
                if (ifp->if_mtu != ifr->ifr_mtu) {
                        ifp->if_mtu = ifr->ifr_mtu;
                        //if running
                        if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
                                //printf("set mtu when running\n");
                                re_stop(sc);

                                re_release_buf(sc);
                                set_rxbufsize(sc);
                                error = re_alloc_buf(sc);

                                if (error == 0) {
                                        re_init_locked(sc);
                                }
                        } else {
                                //if not running
                                re_release_buf(sc);
                                set_rxbufsize(sc);
                                error = re_alloc_buf(sc);
                                if (error == 0) {
                                        /* Init descriptors. */
                                        re_var_init(sc);
                                }

                        }

                        if (ifp->if_mtu > ETHERMTU) {
                                ifp->if_capenable &= ~(IFCAP_TSO |
                                                       IFCAP_VLAN_HWTSO);
                                ifp->if_hwassist &= ~CSUM_TSO;
                        }
                        //	printf("after mtu =%d\n",(int)ifp->if_mtu);
                }
                RE_UNLOCK(sc);
                break;
        case SIOCSIFFLAGS:
                RE_LOCK(sc);
                if (ifp->if_flags & IFF_UP) {
                        re_init_locked(sc);
                } else if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
                        re_stop(sc);
                }
                error = 0;
                RE_UNLOCK(sc);
                break;
        case SIOCADDMULTI:
        case SIOCDELMULTI:
                RE_LOCK(sc);
                re_set_rx_packet_filter(sc);
                RE_UNLOCK(sc);
                error = 0;
                break;
        case SIOCGIFMEDIA:
        case SIOCSIFMEDIA:
                error = ifmedia_ioctl(ifp, ifr, &sc->media, command);
                break;
        case SIOCSIFCAP:
                mask = ifr->ifr_reqcap ^ ifp->if_capenable;
                reinit = 0;

                if ((mask & IFCAP_TXCSUM) != 0 && (ifp->if_capabilities & IFCAP_TXCSUM) != 0) {
                        ifp->if_capenable ^= IFCAP_TXCSUM;
                        if ((ifp->if_capenable & IFCAP_TXCSUM) != 0)  {
                                if ((sc->re_type == MACFG_24) || (sc->re_type == MACFG_25) || (sc->re_type == MACFG_26))
                                        ifp->if_hwassist |= CSUM_TCP | CSUM_UDP;
                                else
                                        ifp->if_hwassist |= RE_CSUM_FEATURES_IPV4;
                        } else
                                ifp->if_hwassist &= ~RE_CSUM_FEATURES_IPV4;
                        reinit = 1;
                }

                if ((mask & IFCAP_TXCSUM_IPV6) != 0 && (ifp->if_capabilities & IFCAP_TXCSUM_IPV6) != 0) {
                        ifp->if_capenable ^= IFCAP_TXCSUM_IPV6;
                        if ((ifp->if_capenable & IFCAP_TXCSUM_IPV6) != 0)  {
                                ifp->if_hwassist |= RE_CSUM_FEATURES_IPV6;
                        } else
                                ifp->if_hwassist &= ~RE_CSUM_FEATURES_IPV6;

                        if (!((sc->re_if_flags & RL_FLAG_DESCV2) &&
                              (sc->re_if_flags & RL_FLAG_8168G_PLUS)))
                                ifp->if_hwassist &= ~RE_CSUM_FEATURES_IPV6;
                        reinit = 1;
                }

                if ((mask & IFCAP_RXCSUM) != 0 &&
                    (ifp->if_capabilities & IFCAP_RXCSUM) != 0) {
                        ifp->if_capenable ^= IFCAP_RXCSUM;
                        reinit = 1;
                }

                if ((mask & IFCAP_RXCSUM_IPV6) != 0 &&
                    (ifp->if_capabilities & IFCAP_RXCSUM_IPV6) != 0) {
                        ifp->if_capenable ^= IFCAP_RXCSUM_IPV6;
                        reinit = 1;
                }

                if ((ifp->if_mtu <= ETHERMTU) || ((sc->re_type>= MACFG_3) &&(sc->re_type <=MACFG_6)) || ((sc->re_type>= MACFG_21) && (sc->re_type <=MACFG_23))) {
                        if (ifp->if_capenable & IFCAP_TXCSUM)
                                sc->re_tx_cstag = 1;
                        else
                                sc->re_tx_cstag = 0;

                        if (ifp->if_capenable & (IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6))
                                sc->re_rx_cstag = 1;
                        else
                                sc->re_rx_cstag = 0;
                }

                if ((mask & IFCAP_TSO4) != 0 &&
                    (ifp->if_capabilities & IFCAP_TSO4) != 0) {
                        ifp->if_capenable ^= IFCAP_TSO4;
                        if ((IFCAP_TSO4 & ifp->if_capenable) != 0)
                                ifp->if_hwassist |= CSUM_IP_TSO;
                        else
                                ifp->if_hwassist &= ~CSUM_IP_TSO;
                        if (ifp->if_mtu > ETHERMTU) {
                                ifp->if_capenable &= ~IFCAP_TSO4;
                                ifp->if_hwassist &= ~CSUM_IP_TSO;
                        }
                }
                /*
                if ((mask & IFCAP_TSO6) != 0 &&
                (ifp->if_capabilities & IFCAP_TSO6) != 0) {
                ifp->if_capenable ^= IFCAP_TSO6;
                if ((IFCAP_TSO6 & ifp->if_capenable) != 0)
                ifp->if_hwassist |= CSUM_IP6_TSO;
                else
                ifp->if_hwassist &= ~CSUM_IP6_TSO;
                if (ifp->if_mtu > ETHERMTU) {
                ifp->if_capenable &= ~IFCAP_TSO6;
                ifp->if_hwassist &= ~CSUM_IP6_TSO;
                }
                if (ifp->if_mtu > ETHERMTU) {
                ifp->if_capenable &= ~IFCAP_TSO6;
                ifp->if_hwassist &= ~CSUM_IP6_TSO;
                }
                }
                */
                if ((mask & IFCAP_VLAN_HWTSO) != 0 &&
                    (ifp->if_capabilities & IFCAP_VLAN_HWTSO) != 0)
                        ifp->if_capenable ^= IFCAP_VLAN_HWTSO;
                if ((mask & IFCAP_VLAN_HWTAGGING) != 0 &&
                    (ifp->if_capabilities & IFCAP_VLAN_HWTAGGING) != 0) {
                        ifp->if_capenable ^= IFCAP_VLAN_HWTAGGING;
                        /* TSO over VLAN requires VLAN hardware tagging. */
                        //if ((ifp->if_capenable & IFCAP_VLAN_HWTAGGING) == 0)
                        //	ifp->if_capenable &= ~IFCAP_VLAN_HWTSO;
                        reinit = 1;
                }
                if (mask & IFCAP_LRO)
                        ifp->if_capenable ^= IFCAP_LRO;

                if ((mask & IFCAP_WOL) != 0 &&
                    (ifp->if_capabilities & IFCAP_WOL) != 0) {
                        if ((mask & IFCAP_WOL_UCAST) != 0)
                                ifp->if_capenable ^= IFCAP_WOL_UCAST;
                        if ((mask & IFCAP_WOL_MCAST) != 0)
                                ifp->if_capenable ^= IFCAP_WOL_MCAST;
                        if ((mask & IFCAP_WOL_MAGIC) != 0)
                                ifp->if_capenable ^= IFCAP_WOL_MAGIC;
                }
                if (reinit && ifp->if_drv_flags & IFF_DRV_RUNNING) {
                        ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
                        re_init(sc);
                }
                VLAN_CAPABILITIES(ifp);
                break;
        default:
                error = EINVAL;
                break;
        }

        /*(void)splx(s);*/

        return(error);
}

static void re_link_on_patch(struct re_softc *sc)
{
        struct ifnet		*ifp;

        ifp = RE_GET_IFNET(sc);

        if (sc->re_type == MACFG_50 || sc->re_type == MACFG_51 || sc->re_type == MACFG_52) {
                if (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_1000MF) {
                        re_eri_write(sc, 0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
                        re_eri_write(sc, 0x1dc, 4, 0x0000001f, ERIAR_ExGMAC);
                } else if (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_100M) {
                        re_eri_write(sc, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                        re_eri_write(sc, 0x1dc, 4, 0x0000001f, ERIAR_ExGMAC);
                } else {
                        re_eri_write(sc, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                        re_eri_write(sc, 0x1dc, 4, 0x0000002d, ERIAR_ExGMAC);
                }
        } else if ((sc->re_type == MACFG_38 || sc->re_type == MACFG_39) && (ifp->if_flags & IFF_UP)) {
                if (sc->re_type == MACFG_38 && (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_10M)) {
                        CSR_WRITE_4(sc, RE_RXCFG, CSR_READ_4(sc, RE_RXCFG) | RE_RXCFG_RX_ALLPHYS);
                } else if (sc->re_type == MACFG_39) {
                        if (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_1000MF) {
                                re_eri_write(sc, 0x1bc, 4, 0x00000011, ERIAR_ExGMAC);
                                re_eri_write(sc, 0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
                        } else if (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_100M) {
                                re_eri_write(sc, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                                re_eri_write(sc, 0x1dc, 4, 0x00000005, ERIAR_ExGMAC);
                        } else {
                                re_eri_write(sc, 0x1bc, 4, 0x0000001f, ERIAR_ExGMAC);
                                re_eri_write(sc, 0x1dc, 4, 0x0000003f, ERIAR_ExGMAC);
                        }
                }
        } else if ((sc->re_type == MACFG_36 || sc->re_type == MACFG_37) && eee_enable == 1) {
                /*Full -Duplex  mode*/
                if (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_FULL_DUP) {
                        re_mdio_write(sc, 0x1F, 0x0006);
                        re_mdio_write(sc, 0x00, 0x5a30);
                        re_mdio_write(sc, 0x1F, 0x0000);
                        if (CSR_READ_1(sc, RE_PHY_STATUS) & (RL_PHY_STATUS_10M | RL_PHY_STATUS_100M))
                                CSR_WRITE_4(sc, RE_TXCFG, (CSR_READ_4(sc, RE_TXCFG) & ~BIT_19) | BIT_25);

                } else {
                        re_mdio_write(sc, 0x1F, 0x0006);
                        re_mdio_write(sc, 0x00, 0x5a00);
                        re_mdio_write(sc, 0x1F, 0x0000);
                        if (CSR_READ_1(sc, RE_PHY_STATUS) & (RL_PHY_STATUS_10M | RL_PHY_STATUS_100M))
                                CSR_WRITE_4(sc, RE_TXCFG, (CSR_READ_4(sc, RE_TXCFG) & ~BIT_19) | RE_TXCFG_IFG);
                }
        } else if ((sc->re_type == MACFG_56 || sc->re_type == MACFG_57 ||
                    sc->re_type == MACFG_58 || sc->re_type == MACFG_59 ||
                    sc->re_type == MACFG_60 || sc->re_type == MACFG_61 ||
                    sc->re_type == MACFG_62 || sc->re_type == MACFG_67 ||
                    sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                    sc->re_type == MACFG_70 || sc->re_type == MACFG_71 ||
                    sc->re_type == MACFG_72 || sc->re_type == MACFG_73 ||
                    sc->re_type == MACFG_74 || sc->re_type == MACFG_75 ||
                    sc->re_type == MACFG_76 || sc->re_type == MACFG_80 ||
                    sc->re_type == MACFG_81 || sc->re_type == MACFG_82 ||
                    sc->re_type == MACFG_83 || sc->re_type == MACFG_84 ||
                    sc->re_type == MACFG_85 || sc->re_type == MACFG_86 ||
                    sc->re_type == MACFG_87 || sc->re_type == MACFG_90 ||
                    sc->re_type == MACFG_91 || sc->re_type == MACFG_92) &&
                   (ifp->if_flags & IFF_UP)) {
                if (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_FULL_DUP)
                        CSR_WRITE_4(sc, RE_TXCFG, (CSR_READ_4(sc, RE_TXCFG) | (BIT_24 | BIT_25)) & ~BIT_19);
                else
                        CSR_WRITE_4(sc, RE_TXCFG, (CSR_READ_4(sc, RE_TXCFG) | BIT_25) & ~(BIT_19 | BIT_24));
        }

        if (sc->re_type == MACFG_56 || sc->re_type == MACFG_57 ||
            sc->re_type == MACFG_61 || sc->re_type == MACFG_62) {
                /*half mode*/
                if (!(CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_FULL_DUP)) {
                        re_mdio_write(sc, 0x1F, 0x0000);
                        re_mdio_write(sc, MII_ANAR, re_mdio_read(sc, MII_ANAR)&~(ANAR_FC |ANAR_PAUSE_ASYM));
                }
        }

        if (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_10M) {
                if (sc->re_type == MACFG_70 || sc->re_type == MACFG_71 ||
                    sc->re_type == MACFG_72 || sc->re_type == MACFG_73) {
                        uint32_t Data32;

                        Data32 = re_eri_read(sc, 0x1D0, 1, ERIAR_ExGMAC);
                        Data32 |= BIT_1;
                        re_eri_write(sc, 0x1D0, 1, Data32, ERIAR_ExGMAC);
                } else if (sc->re_type == MACFG_80 || sc->re_type == MACFG_81 ||
                           sc->re_type == MACFG_82 || sc->re_type == MACFG_83 ||
                           sc->re_type == MACFG_84 || sc->re_type == MACFG_85 ||
                           sc->re_type == MACFG_90 || sc->re_type == MACFG_91 ||
                           sc->re_type == MACFG_92) {
                        re_mac_ocp_write(sc, 0xE080, re_mac_ocp_read(sc, 0xE080)|BIT_1);
                }
        }

        if (sc->RequiredPfmPatch)
                re_set_pfm_patch(sc, (CSR_READ_1(sc, RE_PHY_STATUS) & RL_PHY_STATUS_10M) ? 1 : 0);

        re_init_unlock(sc);
}

static void re_link_down_patch(struct re_softc *sc)
{
        if (sc->RequiredPfmPatch)
                re_set_pfm_patch(sc, 1);

        re_txeof(sc);
        re_rxeof(sc);
        re_stop(sc);
}

/*
 * Check Link Status.
 */
static void re_check_link_status(struct re_softc *sc)
{
        u_int8_t	link_state;
        struct ifnet		*ifp;

        ifp = RE_GET_IFNET(sc);

        if (re_link_ok(sc)) {
                link_state = LINK_STATE_UP;
        } else {
                link_state = LINK_STATE_DOWN;
        }

        if (link_state != sc->link_state) {
                sc->link_state = link_state;
                if (link_state == LINK_STATE_UP) {
                        re_link_on_patch(sc);
                        re_link_state_change(ifp, LINK_STATE_UP);
                } else {
                        re_link_state_change(ifp, LINK_STATE_DOWN);
                        re_link_down_patch(sc);
                }
        }
}

static void re_init_timer(struct re_softc *sc)
{
#ifdef RE_USE_NEW_CALLOUT_FUN
        callout_init(&sc->re_stat_ch, CALLOUT_MPSAFE);
#else
        callout_handle_init(&sc->re_stat_ch);
#endif
}

static void re_stop_timer(struct re_softc *sc)
{
#ifdef RE_USE_NEW_CALLOUT_FUN
        callout_stop(&sc->re_stat_ch);
#else
        untimeout(re_tick, sc, sc->re_stat_ch);
#endif
}

static void re_start_timer(struct re_softc *sc)
{
#ifdef RE_USE_NEW_CALLOUT_FUN
        callout_reset(&sc->re_stat_ch, hz, re_tick, sc);
#else
        re_stop_timer(sc);
        sc->re_stat_ch = timeout(re_tick, sc, hz);
#endif
}

static void re_tick(void *xsc)
{
        /*called per second*/
        struct re_softc		*sc;
        int			s;

        s = splimp();

        sc = xsc;
        /*mii = device_get_softc(sc->re_miibus);

        mii_tick(mii);*/

        splx(s);

        RE_LOCK(sc);

        if (sc->re_link_chg_det == 1) {
                re_check_link_status(sc);
                re_start_timer(sc);
        }

        RE_UNLOCK(sc);

        return;
}

#if OS_VER < VERSION(7,0)
static void re_watchdog(ifp)
struct ifnet		*ifp;
{
        struct re_softc		*sc;

        sc = ifp->if_softc;

        printf("re%d: watchdog timeout\n", sc->re_unit);
#if OS_VER < VERSION(11,0)
        ifp->if_oerrors++;
#else
        if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
#endif

        re_txeof(sc);
        re_rxeof(sc);
        re_init(sc);

        return;
}
#endif

/*
 * Set media options.
 */
static int re_ifmedia_upd(struct ifnet *ifp)
{
        struct re_softc	*sc = ifp->if_softc;
        struct ifmedia	*ifm = &sc->media;
        int anar;
        int gbcr;

        if (IFM_TYPE(ifm->ifm_media) != IFM_ETHER)
                return(EINVAL);

        if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
            sc->re_type == MACFG_70 || sc->re_type == MACFG_71 ||
            sc->re_type == MACFG_72 || sc->re_type == MACFG_73 ||
            sc->re_type == MACFG_74 || sc->re_type == MACFG_75) {
                //Disable Giga Lite
                re_mdio_write(sc, 0x1F, 0x0A42);
                re_clear_eth_phy_bit(sc, 0x14, BIT_9);
                if (sc->re_type == MACFG_70 || sc->re_type == MACFG_71 ||
                    sc->re_type == MACFG_72 || sc->re_type == MACFG_73)
                        re_clear_eth_phy_bit(sc, 0x14, BIT_7);
                re_mdio_write(sc, 0x1F, 0x0A40);
                re_mdio_write(sc, 0x1F, 0x0000);
        }


        switch (IFM_SUBTYPE(ifm->ifm_media)) {
        case IFM_AUTO:
                anar = ANAR_TX_FD |
                       ANAR_TX |
                       ANAR_10_FD |
                       ANAR_10;
                gbcr = GTCR_ADV_1000TFDX |
                       GTCR_ADV_1000THDX;
                break;
        case IFM_1000_SX:
#if OS_VER < 500000
        case IFM_1000_TX:
#else
        case IFM_1000_T:
#endif
                anar = ANAR_TX_FD |
                       ANAR_TX |
                       ANAR_10_FD |
                       ANAR_10;
                gbcr = GTCR_ADV_1000TFDX |
                       GTCR_ADV_1000THDX;
                break;
        case IFM_100_TX:
                gbcr = re_mdio_read(sc, MII_100T2CR) &
                       ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
                if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX) {
                        anar = ANAR_TX_FD |
                               ANAR_TX |
                               ANAR_10_FD |
                               ANAR_10;
                } else {
                        anar = ANAR_TX |
                               ANAR_10_FD |
                               ANAR_10;
                }
                break;
        case IFM_10_T:
                gbcr = re_mdio_read(sc, MII_100T2CR) &
                       ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
                if ((ifm->ifm_media & IFM_GMASK) == IFM_FDX) {
                        anar = ANAR_10_FD |
                               ANAR_10;
                } else {
                        anar = ANAR_10;
                }

                if (sc->re_type == MACFG_13) {
                        re_mdio_write(sc, MII_BMCR, 0x8000);
                }

                break;
        default:
                printf("re%d: Unsupported media type\n", sc->re_unit);
                return(0);
        }

        if (sc->re_device_id==RT_DEVICEID_8162)
                re_clear_eth_ocp_phy_bit(sc, 0xA5D4, RTK_ADVERTISE_2500FULL);

        re_mdio_write(sc, 0x1F, 0x0000);
        if (sc->re_device_id==RT_DEVICEID_8169 || sc->re_device_id==RT_DEVICEID_8169SC ||
            sc->re_device_id==RT_DEVICEID_8168 || sc->re_device_id==RT_DEVICEID_8161 ||
            sc->re_device_id==RT_DEVICEID_8162) {
                re_mdio_write(sc, MII_ANAR, anar | ANAR_FC | ANAR_PAUSE_ASYM);
                re_mdio_write(sc, MII_100T2CR, gbcr);
                re_mdio_write(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN | BMCR_STARTNEG);
        } else if (sc->re_type == MACFG_36) {
                re_mdio_write(sc, MII_ANAR, anar | ANAR_FC | ANAR_PAUSE_ASYM);
                re_mdio_write(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN | BMCR_STARTNEG);
        } else {
                re_mdio_write(sc, MII_ANAR, anar | 1);
                re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
        }

        return(0);
}

/*
 * Report current media status.
 */
static void re_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
        struct re_softc		*sc;

        sc = ifp->if_softc;

        RE_LOCK(sc);

        ifmr->ifm_status = IFM_AVALID;
        ifmr->ifm_active = IFM_ETHER;

        if (re_link_ok(sc)) {
                unsigned char msr;

                ifmr->ifm_status |= IFM_ACTIVE;

                msr = CSR_READ_1(sc, RE_PHY_STATUS);
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
        }

        RE_UNLOCK(sc);

        return;
}

static bool re_is_advanced_eee_enabled(struct re_softc *sc)
{
        switch (sc->re_type) {
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                if (re_real_ocp_phy_read(sc, 0xA430) & BIT_15)
                        return true;
                break;
        default:
                break;
        }

        return false;
}

static void _re_disable_advanced_eee(struct re_softc *sc)
{
        u_int16_t data;
        bool lock;

        if (re_is_advanced_eee_enabled(sc))
                lock = true;
        else
                lock = false;

        if (lock)
                re_set_phy_mcu_patch_request(sc);

        switch (sc->re_type) {
        case MACFG_59:
                re_eri_write(sc, 0x1EA, 1, 0x00, ERIAR_ExGMAC);

                re_mdio_write(sc, 0x1F, 0x0A42);
                data = re_mdio_read(sc, 0x16);
                data &= ~(BIT_1);
                re_mdio_write(sc, 0x16, data);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;
        case MACFG_60:
                data = re_mac_ocp_read(sc, 0xE052);
                data &= ~(BIT_0);
                re_mac_ocp_write(sc, 0xE052, data);

                re_mdio_write(sc, 0x1F, 0x0A42);
                data = re_mdio_read(sc, 0x16);
                data &= ~(BIT_1);
                re_mdio_write(sc, 0x16, data);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
                data = re_mac_ocp_read(sc, 0xE052);
                data &= ~(BIT_0);
                re_mac_ocp_write(sc, 0xE052, data);
                break;
        case MACFG_68:
        case MACFG_69:
        case MACFG_74:
        case MACFG_75:
                data = re_mac_ocp_read(sc, 0xE052);
                data &= ~(BIT_0);
                re_mac_ocp_write(sc, 0xE052, data);

                re_mdio_write(sc, 0x1F, 0x0A43);
                data = re_mdio_read(sc, 0x10) & ~(BIT_15);
                re_mdio_write(sc, 0x10, data);

                re_mdio_write(sc, 0x1F, 0x0A44);
                data = re_mdio_read(sc, 0x11) & ~(BIT_12 | BIT_13 | BIT_14);
                re_mdio_write(sc, 0x11, data);
                re_mdio_write(sc, 0x1f, 0x0000);
                break;
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_clear_mac_ocp_bit(sc, 0xE052, BIT_0);
                re_clear_eth_ocp_phy_bit(sc, 0xA442, BIT_12 | BIT_13);
                re_clear_eth_ocp_phy_bit(sc, 0xA430, BIT_15);
                break;
        }

        if (lock)
                re_clear_phy_mcu_patch_request(sc);
}

static void re_disable_advanced_eee(struct re_softc *sc)
{
        if (sc->HwSuppDashVer > 1)
                OOB_mutex_lock(sc);

        _re_disable_advanced_eee(sc);

        if (sc->HwSuppDashVer > 1)
                OOB_mutex_unlock(sc);
}

static int re_enable_eee(struct re_softc *sc)
{
        int ret;
        u_int16_t data;

        ret = 0;
        switch (sc->re_type) {
        case MACFG_42:
        case MACFG_43:
                re_eri_write(sc, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0004);
                if (CSR_READ_1(sc, 0xEF) & 0x02) {
                        re_mdio_write(sc, 0x10, 0x731F);
                        re_mdio_write(sc, 0x19, 0x7630);
                } else {
                        re_mdio_write(sc, 0x10, 0x711F);
                        re_mdio_write(sc, 0x19, 0x7030);
                }
                re_mdio_write(sc, 0x1A, 0x1506);
                re_mdio_write(sc, 0x1B, 0x0551);
                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0002);
                re_mdio_write(sc, 0x0D, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0003);
                re_mdio_write(sc, 0x0E, 0x0015);
                re_mdio_write(sc, 0x0D, 0x4003);
                re_mdio_write(sc, 0x0E, 0x0002);
                re_mdio_write(sc, 0x0D, 0x0000);

                re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
                break;

        case MACFG_53:
        case MACFG_54:
        case MACFG_55:
                re_eri_write(sc, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0004);
                re_mdio_write(sc, 0x10, 0x731F);
                re_mdio_write(sc, 0x19, 0x7630);
                re_mdio_write(sc, 0x1A, 0x1506);
                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0002);
                re_mdio_write(sc, 0x0D, 0x0000);

                re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
                break;

        case MACFG_36:
        case MACFG_37:
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
                if ((CSR_READ_1(sc, RE_CFG4)&RL_CFG4_CUSTOMIZED_LED) && (CSR_READ_1(sc, RE_MACDBG) & BIT_7)) {
                        re_mdio_write(sc, 0x1F, 0x0005);
                        re_mdio_write(sc, 0x05, 0x8AC8);
                        re_mdio_write(sc, 0x06, CSR_READ_1(sc, RE_CUSTOM_LED));
                        re_mdio_write(sc, 0x05, 0x8B82);
                        data = re_mdio_read(sc, 0x06) | 0x0010;
                        re_mdio_write(sc, 0x05, 0x8B82);
                        re_mdio_write(sc, 0x06, data);
                        re_mdio_write(sc, 0x1F, 0x0000);
                }
                break;

        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
                data = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC) | 0x0003;
                re_eri_write(sc, 0x1B0, 4, data, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0007);
                re_mdio_write(sc, 0x1E, 0x0020);
                data = re_mdio_read(sc, 0x15)|0x0100;
                re_mdio_write(sc, 0x15, data);
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B85);
                data = re_mdio_read(sc, 0x06)|0x2000;
                re_mdio_write(sc, 0x06, data);
                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0006);
                re_mdio_write(sc, 0x1D, 0x0000);
                break;

        case MACFG_38:
        case MACFG_39:
                data = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                data |= BIT_1 | BIT_0;
                re_eri_write(sc, 0x1B0, 4, data, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0004);
                re_mdio_write(sc, 0x1F, 0x0007);
                re_mdio_write(sc, 0x1e, 0x0020);
                data = re_mdio_read(sc, 0x15);
                data |= BIT_8;
                re_mdio_write(sc, 0x15, data);
                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B85);
                data = re_mdio_read(sc, 0x06);
                data |= BIT_13;
                re_mdio_write(sc, 0x06, data);
                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0006);
                re_mdio_write(sc, 0x0D, 0x0000);
                break;

        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
                data = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                data |= BIT_1 | BIT_0;
                re_eri_write(sc, 0x1B0, 4, data, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0A43);
                data = re_mdio_read(sc, 0x11);
                re_mdio_write(sc, 0x11, data | BIT_4);
                re_mdio_write(sc, 0x1F, 0x0A5D);
                re_mdio_write(sc, 0x10, 0x0006);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;

        case MACFG_80:
        case MACFG_81:
                re_set_mac_ocp_bit(sc, 0xE040, (BIT_1|BIT_0));
                re_set_mac_ocp_bit(sc, 0xEB62, (BIT_2|BIT_1));

                re_set_eth_ocp_phy_bit(sc, 0xA432, BIT_4);
                re_set_eth_ocp_phy_bit(sc, 0xA5D0, (BIT_2 | BIT_1));
                re_clear_eth_ocp_phy_bit(sc, 0xA6D4, BIT_0);

                re_clear_eth_ocp_phy_bit(sc, 0xA6D8, BIT_4);
                re_clear_eth_ocp_phy_bit(sc, 0xA428, BIT_7);
                re_clear_eth_ocp_phy_bit(sc, 0xA4A2, BIT_9);
                break;

        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
                re_set_mac_ocp_bit(sc, 0xE040, (BIT_1|BIT_0));

                re_set_eth_ocp_phy_bit(sc, 0xA432, BIT_4);

                re_set_eth_ocp_phy_bit(sc, 0xA5D0, (BIT_2 | BIT_1));
                re_clear_eth_ocp_phy_bit(sc, 0xA6D4, BIT_0);

                re_clear_eth_ocp_phy_bit(sc, 0xA6D8, BIT_4);
                re_clear_eth_ocp_phy_bit(sc, 0xA428, BIT_7);
                re_clear_eth_ocp_phy_bit(sc, 0xA4A2, BIT_9);
                break;

        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_set_mac_ocp_bit(sc, 0xE040, (BIT_1|BIT_0));

                re_set_eth_ocp_phy_bit(sc, 0xA5D0, (BIT_2 | BIT_1));
                re_clear_eth_ocp_phy_bit(sc, 0xA6D4, (BIT_1|BIT_0));

                re_clear_eth_ocp_phy_bit(sc, 0xA428, BIT_7);
                re_clear_eth_ocp_phy_bit(sc, 0xA4A2, BIT_9);
                break;

        default:
                ret = -EOPNOTSUPP;
                break;
        }

        switch (sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_74:
        case MACFG_75:
                re_mdio_write(sc, 0x1F, 0x0A4A);
                re_set_eth_phy_bit(sc, 0x11, BIT_9);
                re_mdio_write(sc, 0x1F, 0x0A42);
                re_set_eth_phy_bit(sc, 0x14, BIT_7);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;
        }

        /*Advanced EEE*/
        re_disable_advanced_eee(sc);

        return ret;
}

static int re_disable_eee(struct re_softc *sc)
{
        int ret;
        u_int16_t data;

        ret = 0;
        switch (sc->re_type) {
        case MACFG_42:
        case MACFG_43:
                re_eri_write(sc, 0x1B0, 2, 0, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0004);
                re_mdio_write(sc, 0x10, 0x401F);
                re_mdio_write(sc, 0x19, 0x7030);

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

                re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
                break;

        case MACFG_53:
                re_eri_write(sc, 0x1B0, 2, 0, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0004);
                re_mdio_write(sc, 0x10, 0x401F);
                re_mdio_write(sc, 0x19, 0x7030);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0000);

                re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
                break;

        case MACFG_54:
        case MACFG_55:
                re_eri_write(sc, 0x1B0, 2, 0, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0004);
                re_mdio_write(sc, 0x10, 0xC07F);
                re_mdio_write(sc, 0x19, 0x7030);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0000);

                re_mdio_write(sc, MII_BMCR, BMCR_AUTOEN | BMCR_STARTNEG);
                break;

        case MACFG_36:
        case MACFG_37:
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
                break;

        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
                data = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC)& ~0x0003;
                re_eri_write(sc, 0x1B0, 4, data, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B85);
                data = re_mdio_read(sc, 0x06) & ~0x2000;
                re_mdio_write(sc, 0x06, data);
                re_mdio_write(sc, 0x1F, 0x0007);
                re_mdio_write(sc, 0x1E, 0x0020);
                data = re_mdio_read(sc, 0x15) & ~0x0100;
                re_mdio_write(sc, 0x15, data);
                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0000);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;

        case MACFG_38:
        case MACFG_39:
                data = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                data &= ~(BIT_1 | BIT_0);
                re_eri_write(sc, 0x1B0, 4, data, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B85);
                data = re_mdio_read(sc, 0x06);
                data &= ~BIT_13;
                re_mdio_write(sc, 0x06, data);
                re_mdio_write(sc, 0x1F, 0x0004);
                re_mdio_write(sc, 0x1F, 0x0007);
                re_mdio_write(sc, 0x1e, 0x0020);
                data = re_mdio_read(sc, 0x15);
                data &= ~BIT_8;
                re_mdio_write(sc, 0x15, data);
                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0007);
                re_mdio_write(sc, 0x0E, 0x003C);
                re_mdio_write(sc, 0x0D, 0x4007);
                re_mdio_write(sc, 0x0E, 0x0000);
                re_mdio_write(sc, 0x0D, 0x0000);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;

        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
                data = re_eri_read(sc, 0x1B0, 4, ERIAR_ExGMAC);
                data &= ~(BIT_1 | BIT_0);
                re_eri_write(sc, 0x1B0, 4, data, ERIAR_ExGMAC);
                re_mdio_write(sc, 0x1F, 0x0A43);
                data = re_mdio_read(sc, 0x11);
                if (sc->re_type == MACFG_75)
                        re_mdio_write(sc, 0x11, data | BIT_4);
                else
                        re_mdio_write(sc, 0x11, data & ~BIT_4);
                re_mdio_write(sc, 0x1F, 0x0A5D);
                re_mdio_write(sc, 0x10, 0x0000);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;

        case MACFG_80:
        case MACFG_81:
                re_clear_mac_ocp_bit(sc, 0xE040, (BIT_1|BIT_0));
                re_clear_mac_ocp_bit(sc, 0xEB62, (BIT_2|BIT_1));

                re_clear_eth_ocp_phy_bit(sc, 0xA432, BIT_4);
                re_clear_eth_ocp_phy_bit(sc, 0xA5D0, (BIT_2 | BIT_1));
                re_clear_eth_ocp_phy_bit(sc, 0xA6D4, BIT_0);

                re_clear_eth_ocp_phy_bit(sc, 0xA6D8, BIT_4);
                re_clear_eth_ocp_phy_bit(sc, 0xA428, BIT_7);
                re_clear_eth_ocp_phy_bit(sc, 0xA4A2, BIT_9);
                break;

        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
                re_clear_mac_ocp_bit(sc, 0xE040, (BIT_1|BIT_0));

                re_set_eth_ocp_phy_bit(sc, 0xA432, BIT_4);

                re_clear_eth_ocp_phy_bit(sc, 0xA5D0, (BIT_2 | BIT_1));
                re_clear_eth_ocp_phy_bit(sc, 0xA6D4, BIT_0);

                re_clear_eth_ocp_phy_bit(sc, 0xA6D8, BIT_4);
                re_clear_eth_ocp_phy_bit(sc, 0xA428, BIT_7);
                re_clear_eth_ocp_phy_bit(sc, 0xA4A2, BIT_9);
                break;

        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_clear_mac_ocp_bit(sc, 0xE040, (BIT_1|BIT_0));

                re_clear_eth_ocp_phy_bit(sc, 0xA5D0, (BIT_2 | BIT_1));
                re_clear_eth_ocp_phy_bit(sc, 0xA6D4, (BIT_0 | BIT_1));

                re_clear_eth_ocp_phy_bit(sc, 0xA428, BIT_7);
                re_clear_eth_ocp_phy_bit(sc, 0xA4A2, BIT_9);
                break;

        default:
                ret = -EOPNOTSUPP;
                break;
        }

        switch (sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_74:
        case MACFG_75:
                re_mdio_write(sc, 0x1F, 0x0A42);
                re_clear_eth_phy_bit(sc, 0x14, BIT_7);
                re_mdio_write(sc, 0x1F, 0x0A4A);
                re_clear_eth_phy_bit(sc, 0x11, BIT_9);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;
        }

        /*Advanced EEE*/
        re_disable_advanced_eee(sc);

        return ret;
}

static void re_init_hw_phy_mcu(struct re_softc *sc)
{
        if (re_hw_phy_mcu_code_ver_matched(sc))
                return;

        switch (sc->re_type) {
        case MACFG_36:
                re_set_phy_mcu_8168e_1(sc);
                break;
        case MACFG_37:
                re_set_phy_mcu_8168e_2(sc);
                break;
        case MACFG_38:
                re_set_phy_mcu_8168evl_1(sc);
                break;
        case MACFG_39:
                re_set_phy_mcu_8168evl_2(sc);
                break;
        case MACFG_50:
                re_set_phy_mcu_8168f_1(sc);
                break;
        case MACFG_51:
                re_set_phy_mcu_8168f_2(sc);
                break;
        case MACFG_52:
                re_set_phy_mcu_8411_1(sc);
                break;
        case MACFG_56:
                re_set_phy_mcu_8168g_1(sc);
                break;
        case MACFG_59:
                re_set_phy_mcu_8168gu_2(sc);
                break;
        case MACFG_60:
                re_set_phy_mcu_8411b_1(sc);
                break;
        case MACFG_61:
                re_set_phy_mcu_8168ep_1(sc);
                break;
        case MACFG_67:
                re_set_phy_mcu_8168ep_2(sc);
                break;
        case MACFG_68:
                re_set_phy_mcu_8168h_1(sc);
                break;
        case MACFG_69:
        case MACFG_76:
                re_set_phy_mcu_8168h_2(sc);
                break;
        case MACFG_74:
                re_set_phy_mcu_8168h_3(sc);
                break;
        case MACFG_75:
                break;
        case MACFG_80:
                re_set_phy_mcu_8125a_1(sc);
                break;
        case MACFG_81:
                re_set_phy_mcu_8125a_2(sc);
                break;
        case MACFG_82:
                re_set_phy_mcu_8125b_1(sc);
                break;
        case MACFG_83:
                re_set_phy_mcu_8125b_2(sc);
                break;
        case MACFG_84:
                re_set_phy_mcu_8125bp_1(sc);
                break;
        case MACFG_85:
                //do nothing
                break;
        case MACFG_86:
                re_set_phy_mcu_8125d_1(sc);
                break;
        case MACFG_87:
                //do nothing
                break;
        case MACFG_90:
                re_set_phy_mcu_8126a_1(sc);
                break;
        case MACFG_91:
                re_set_phy_mcu_8126a_2(sc);
                break;
        case MACFG_92:
                re_set_phy_mcu_8126a_3(sc);
                break;
        }

        re_write_hw_phy_mcu_code_ver(sc);

        re_mdio_write(sc, 0x1F, 0x0000);
}

static void re_set_hw_phy_before_init_phy_mcu(struct re_softc *sc)
{
        device_t dev = sc->dev;
        u_int16_t PhyRegValue;

        switch (sc->re_type) {
        case MACFG_82:
                re_real_ocp_phy_write(sc, 0xBF86, 0x9000);

                re_set_eth_ocp_phy_bit(sc, 0xC402, BIT_10);
                re_clear_eth_ocp_phy_bit(sc, 0xC402, BIT_10);

                PhyRegValue = re_real_ocp_phy_read(sc, 0xBF86);
                PhyRegValue &= (BIT_1 | BIT_0);
                if (PhyRegValue != 0)
                        device_printf(dev, "PHY watch dog not clear, value = 0x%x \n", PhyRegValue);

                re_real_ocp_phy_write(sc, 0xBD86, 0x1010);
                re_real_ocp_phy_write(sc, 0xBD88, 0x1010);

                re_clear_set_eth_ocp_phy_bit(sc,
                                             0xBD4E,
                                             BIT_11 | BIT_10,
                                             BIT_11);
                re_clear_set_eth_ocp_phy_bit(sc,
                                             0xBF46,
                                             BIT_11 | BIT_10 | BIT_9 | BIT_8,
                                             BIT_10 | BIT_9 | BIT_8);
                break;
        }
}

static void re_hw_phy_config(struct re_softc *sc)
{

        switch (sc->re_type) {
        case MACFG_59:
        case MACFG_60:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
        case MACFG_80:
        case MACFG_81:
                re_disable_ocp_phy_power_saving(sc);
                break;
        }

        if (HW_DASH_SUPPORT_TYPE_3(sc) && sc->HwPkgDet == 0x06)
                return;

        re_set_hw_phy_before_init_phy_mcu(sc);

        if (FALSE == re_phy_ram_code_check(sc)) {
                re_set_phy_ram_code_check_fail_flag(sc);
                return;
        }

        re_init_hw_phy_mcu(sc);

        re_mdio_write(sc, 0x1F, 0x0000);

        if (sc->re_type == MACFG_3) {
                re_hw_phy_config_8169_8110s(sc);
        } else if (sc->re_type == MACFG_4) {
                re_hw_phy_config_8169_8110sb(sc);
        } else if (sc->re_type == MACFG_5) {
                re_hw_phy_config_8169_8110sc(sc);
        } else if (sc->re_type == MACFG_6) {
                re_mdio_write(sc, 0x1f, 0x0001);
                re_mdio_write(sc, 0x04, 0x0000);
                re_mdio_write(sc, 0x03, 0x00a1);
                re_mdio_write(sc, 0x02, 0x0008);
                re_mdio_write(sc, 0x01, 0x0120);
                re_mdio_write(sc, 0x00, 0x1000);
                re_mdio_write(sc, 0x04, 0x0800);

                re_mdio_write(sc, 0x04, 0x9000);
                re_mdio_write(sc, 0x03, 0x802f);
                re_mdio_write(sc, 0x02, 0x4f02);
                re_mdio_write(sc, 0x01, 0x0409);
                re_mdio_write(sc, 0x00, 0xf099);
                re_mdio_write(sc, 0x04, 0x9800);

                re_mdio_write(sc, 0x04, 0xa000);
                re_mdio_write(sc, 0x03, 0xdf01);
                re_mdio_write(sc, 0x02, 0xdf20);
                re_mdio_write(sc, 0x01, 0xff95);
                re_mdio_write(sc, 0x00, 0xba00);
                re_mdio_write(sc, 0x04, 0xa800);

                re_mdio_write(sc, 0x04, 0xf000);
                re_mdio_write(sc, 0x03, 0xdf01);
                re_mdio_write(sc, 0x02, 0xdf20);
                re_mdio_write(sc, 0x01, 0x101a);
                re_mdio_write(sc, 0x00, 0xa0ff);
                re_mdio_write(sc, 0x04, 0xf800);
                re_mdio_write(sc, 0x04, 0x0000);
                re_mdio_write(sc, 0x1f, 0x0000);

                re_mdio_write(sc, 0x1f, 0x0001);
                re_mdio_write(sc, 0x0b, 0x8480);
                re_mdio_write(sc, 0x1f, 0x0000);

                re_mdio_write(sc, 0x1f, 0x0001);
                re_mdio_write(sc, 0x18, 0x67c7);
                re_mdio_write(sc, 0x04, 0x2000);
                re_mdio_write(sc, 0x03, 0x002f);
                re_mdio_write(sc, 0x02, 0x4360);
                re_mdio_write(sc, 0x01, 0x0109);
                re_mdio_write(sc, 0x00, 0x3022);
                re_mdio_write(sc, 0x04, 0x2800);
                re_mdio_write(sc, 0x1f, 0x0000);

                re_mdio_write(sc, 0x1f, 0x0001);
                re_mdio_write(sc, 0x17, 0x0CC0);
        } else if (sc->re_type == MACFG_14) {
                re_mdio_write(sc, 0x1f, 0x0000);
                re_mdio_write(sc, 0x11, re_mdio_read(sc, 0x11) | 0x1000);
                re_mdio_write(sc, 0x19, re_mdio_read(sc, 0x19) | 0x2000);
                re_mdio_write(sc, 0x10, re_mdio_read(sc, 0x10) | 0x8000);

                re_mdio_write(sc, 0x1f, 0x0003);
                re_mdio_write(sc, 0x08, 0x441D);
                re_mdio_write(sc, 0x01, 0x9100);
        } else if (sc->re_type == MACFG_15) {
                re_mdio_write(sc, 0x1f, 0x0000);
                re_mdio_write(sc, 0x11, re_mdio_read(sc, 0x11) | 0x1000);
                re_mdio_write(sc, 0x19, re_mdio_read(sc, 0x19) | 0x2000);
                re_mdio_write(sc, 0x10, re_mdio_read(sc, 0x10) | 0x8000);

                re_mdio_write(sc, 0x1f, 0x0003);
                re_mdio_write(sc, 0x08, 0x441D);
                re_mdio_write(sc, 0x01, 0x9100);
        } else if (sc->re_type == MACFG_17) {
                re_mdio_write(sc, 0x1f, 0x0000);
                re_mdio_write(sc, 0x11, re_mdio_read(sc, 0x11) | 0x1000);
                re_mdio_write(sc, 0x19, re_mdio_read(sc, 0x19) | 0x2000);
                re_mdio_write(sc, 0x10, re_mdio_read(sc, 0x10) | 0x8000);

                re_mdio_write(sc, 0x1f, 0x0003);
                re_mdio_write(sc, 0x08, 0x441D);

                re_mdio_write(sc, 0x1f, 0x0000);
        } else if (sc->re_type == MACFG_21) {
                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x0B, 0x94B0);

                re_mdio_write(sc, 0x1F, 0x0003);
                re_mdio_write(sc, 0x12, 0x6096);
                re_mdio_write(sc, 0x1F, 0x0000);
        } else if (sc->re_type == MACFG_22) {
                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x0B, 0x94B0);

                re_mdio_write(sc, 0x1F, 0x0003);
                re_mdio_write(sc, 0x12, 0x6096);
        } else if (sc->re_type == MACFG_23) {
                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x0B, 0x94B0);

                re_mdio_write(sc, 0x1F, 0x0003);
                re_mdio_write(sc, 0x12, 0x6096);
        } else if (sc->re_type == MACFG_24) {
                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x12, 0x2300);
                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x1F, 0x0003);
                re_mdio_write(sc, 0x16, 0x000A);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0003);
                re_mdio_write(sc, 0x12, 0xC096);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x00, 0x88DE);
                re_mdio_write(sc, 0x01, 0x82B1);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x08, 0x9E30);
                re_mdio_write(sc, 0x09, 0x01F0);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x0A, 0x5500);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x03, 0x7002);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x14, re_mdio_read(sc, 0x14) | BIT_5);
                re_mdio_write(sc, 0x0d, re_mdio_read(sc, 0x0d) | BIT_5);
        } else if (sc->re_type == MACFG_25) {
                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x12, 0x2300);
                re_mdio_write(sc, 0x1F, 0x0003);
                re_mdio_write(sc, 0x16, 0x0F0A);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x00, 0x88DE);
                re_mdio_write(sc, 0x01, 0x82B1);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x0C, 0x7EB8);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x06, 0x0761);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x03, 0x802F);
                re_mdio_write(sc, 0x02, 0x4F02);
                re_mdio_write(sc, 0x01, 0x0409);
                re_mdio_write(sc, 0x00, 0xF099);
                re_mdio_write(sc, 0x04, 0x9800);
                re_mdio_write(sc, 0x04, 0x9000);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x16, re_mdio_read(sc, 0x16) | BIT_0);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x14, re_mdio_read(sc, 0x14) | BIT_5);
                re_mdio_write(sc, 0x0D, re_mdio_read(sc, 0x0D) & ~BIT_5);

                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x1D, 0x3D98);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x17, 0x0CC0);
                re_mdio_write(sc, 0x1F, 0x0000);
        } else if (sc->re_type == MACFG_26) {
                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x12, 0x2300);
                re_mdio_write(sc, 0x1F, 0x0003);
                re_mdio_write(sc, 0x16, 0x0F0A);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x00, 0x88DE);
                re_mdio_write(sc, 0x01, 0x82B1);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x0C, 0x7EB8);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x06, 0x5461);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0002);
                re_mdio_write(sc, 0x06, 0x5461);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x16, re_mdio_read(sc, 0x16) | BIT_0);

                re_mdio_write(sc, 0x1F, 0x0000);
                re_mdio_write(sc, 0x14, re_mdio_read(sc, 0x14) | BIT_5);
                re_mdio_write(sc, 0x0D, re_mdio_read(sc, 0x0D) & ~BIT_5);

                re_mdio_write(sc, 0x1F, 0x0001);
                re_mdio_write(sc, 0x1D, 0x3D98);
                re_mdio_write(sc, 0x1F, 0x0000);

                re_mdio_write(sc, 0x1f, 0x0001);
                re_mdio_write(sc, 0x17, 0x0CC0);
                re_mdio_write(sc, 0x1F, 0x0000);
        } else if (sc->re_type == MACFG_27) {
                re_hw_phy_config_macfg27(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_28) {
                re_hw_phy_config_macfg28(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_31) {
                re_hw_phy_config_macfg31(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_32) {
                re_hw_phy_config_macfg32(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_33) {
                re_hw_phy_config_macfg33(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_36 || sc->re_type == MACFG_37) {
                re_hw_phy_config_macfg36(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_38) {
                re_hw_phy_config_macfg38(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_39) {
                re_hw_phy_config_macfg39(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_41) {
                re_hw_phy_config_macfg41(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_42 || sc->re_type == MACFG_43) {
                re_hw_phy_config_macfg42(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_50) {
                re_hw_phy_config_macfg50(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_51) {
                re_hw_phy_config_macfg51(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_52) {
                re_hw_phy_config_macfg52(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_53) {
                re_hw_phy_config_macfg53(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_54 || sc->re_type == MACFG_55) {
                re_hw_phy_config_macfg54(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_56) {
                re_hw_phy_config_macfg56(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_58) {
                re_hw_phy_config_macfg58(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_59) {
                re_hw_phy_config_macfg59(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_60) {
                re_hw_phy_config_macfg60(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_61) {
                re_hw_phy_config_macfg61(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_62 || sc->re_type == MACFG_67) {
                re_hw_phy_config_macfg62(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_63) {
                re_hw_phy_config_macfg63(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_64) {
                re_hw_phy_config_macfg64(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_65) {
                re_hw_phy_config_macfg65(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_66) {
                re_hw_phy_config_macfg66(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_68) {
                re_hw_phy_config_8168h_1(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_69 || sc->re_type == MACFG_76) {
                re_hw_phy_config_8168h_2(sc, phy_power_saving);
        }  else if (sc->re_type == MACFG_70 || sc->re_type == MACFG_71 ||
                    sc->re_type == MACFG_72 || sc->re_type == MACFG_73) {
            re_hw_phy_config_8168fp_1(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_74) {
                re_hw_phy_config_8168h_3(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_75) {
                re_hw_phy_config_8168h_4(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_80) {
                re_hw_phy_config_8125a_1(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_81) {
                re_hw_phy_config_8125a_2(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_82) {
                re_hw_phy_config_8125b_1(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_83) {
                re_hw_phy_config_8125b_2(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_84) {
                re_hw_phy_config_8125bp_1(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_85) {
                re_hw_phy_config_8125bp_2(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_86) {
                re_hw_phy_config_8125d_1(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_87) {
                re_hw_phy_config_8125d_2(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_90) {
                re_hw_phy_config_8126a_1(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_91) {
                re_hw_phy_config_8126a_2(sc, phy_power_saving);
        } else if (sc->re_type == MACFG_92) {
                re_hw_phy_config_8126a_3(sc, phy_power_saving);
        }

#ifdef ENABLE_FIBER_SUPPORT
        if (HW_FIBER_MODE_ENABLED(sc))
                re_hw_fiber_phy_config(sc);
#endif //ENABLE_FIBER_SUPPORT

        //EthPhyPPSW
        if (sc->re_type == MACFG_56 || sc->re_type == MACFG_57 ||
            sc->re_type == MACFG_58 || sc->re_type == MACFG_59 ||
            sc->re_type == MACFG_60) {
                //disable EthPhyPPSW
                re_mdio_write(sc, 0x1F, 0x0BCD);
                re_mdio_write(sc, 0x14, 0x5065);
                re_mdio_write(sc, 0x14, 0xD065);
                re_mdio_write(sc, 0x1F, 0x0BC8);
                re_mdio_write(sc, 0x12, 0x00ED);
                re_mdio_write(sc, 0x1F, 0x0BCD);
                re_mdio_write(sc, 0x14, 0x1065);
                re_mdio_write(sc, 0x14, 0x9065);
                re_mdio_write(sc, 0x14, 0x1065);
                re_mdio_write(sc, 0x1F, 0x0000);
        } else if (sc->re_type == MACFG_68 || sc->re_type == MACFG_69 ||
                   sc->re_type == MACFG_74 || sc->re_type == MACFG_75) {
                //enable EthPhyPPSW
                re_mdio_write(sc, 0x1F, 0x0A44);
                re_set_eth_phy_bit(sc, 0x11, BIT_7);
                re_mdio_write(sc, 0x1F, 0x0000);
        }

        switch(sc->re_type) {
        case MACFG_56:
        case MACFG_57:
        case MACFG_58:
        case MACFG_59:
        case MACFG_60:
        case MACFG_61:
        case MACFG_62:
        case MACFG_67:
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                if (phy_mdix_mode == RE_ETH_PHY_FORCE_MDI) {
                        //Force MDI
                        re_mdio_write(sc, 0x1F, 0x0A43);
                        re_set_eth_phy_bit(sc, 0x10, BIT_8 | BIT_9);
                        re_mdio_write(sc, 0x1F, 0x0000);
                } else if (phy_mdix_mode == RE_ETH_PHY_FORCE_MDIX) {
                        //Force MDIX
                        re_mdio_write(sc, 0x1F, 0x0A43);
                        re_clear_eth_phy_bit(sc, 0x10, BIT_8);
                        re_set_eth_phy_bit(sc, 0x10, BIT_9);
                        re_mdio_write(sc, 0x1F, 0x0000);
                } else {
                        //Auto MDI/MDIX
                        re_mdio_write(sc, 0x1F, 0x0A43);
                        re_clear_eth_phy_bit(sc, 0x10, BIT_8 | BIT_9);
                        re_mdio_write(sc, 0x1F, 0x0000);
                }

                break;
        }

        //legacy force mode(Chap 22)
        switch(sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
        case MACFG_84:
        case MACFG_85:
        case MACFG_86:
        case MACFG_87:
        case MACFG_90:
        case MACFG_91:
        case MACFG_92:
                re_clear_eth_ocp_phy_bit(sc, 0xA5B4, BIT_15);
                break;
        }

        if (phy_power_saving == 1) {
                switch (sc->re_type) {
                case MACFG_59:
                case MACFG_60:
                case MACFG_62:
                case MACFG_67:
                case MACFG_68:
                case MACFG_69:
                case MACFG_70:
                case MACFG_71:
                case MACFG_72:
                case MACFG_73:
                case MACFG_74:
                case MACFG_75:
                case MACFG_76:
                        re_enable_ocp_phy_power_saving(sc);
                        break;
                }
        }

        if (eee_enable == 1)
                re_enable_eee(sc);
        else
                re_disable_eee(sc);

        re_mdio_write(sc, 0x1F, 0x0000);
}

static u_int8_t RtCheckPciEPhyAddr(struct re_softc *sc, int RegAddr)
{
        if (sc->re_type != MACFG_74 && sc->re_type != MACFG_75)
                goto exit;

        if (RegAddr & (BIT_6 | BIT_5))
                re_clear_set_mac_ocp_bit(sc, 0xDE28,
                                         (BIT_1 | BIT_0),
                                         (RegAddr >> 5) & (BIT_1 | BIT_0));

        RegAddr &= 0x1F;

exit:
        return RegAddr;
}

static void _re_ephy_write(struct re_softc *sc, u_int8_t RegAddr, u_int16_t RegData)
{
        u_int32_t		TmpUlong=0x80000000;
        u_int32_t		Timeout=0;

        TmpUlong |= (((u_int32_t)RegAddr<<16) | (u_int32_t)RegData);

        CSR_WRITE_4(sc, RE_EPHYAR, TmpUlong);

        /* Wait for writing to Phy ok */
        for (Timeout=0; Timeout<5; Timeout++) {
                DELAY(1000);
                if ((CSR_READ_4(sc, RE_EPHYAR)&PHYAR_Flag)==0)
                        break;
        }
}

void re_ephy_write(struct re_softc *sc, u_int8_t RegAddr, u_int16_t RegData)
{
        _re_ephy_write(sc, RtCheckPciEPhyAddr(sc, RegAddr), RegData);
}

static u_int16_t _re_ephy_read(struct re_softc *sc, u_int8_t RegAddr)
{
        u_int16_t		RegData;
        u_int32_t		TmpUlong;
        u_int32_t		Timeout=0;

        TmpUlong = ((u_int32_t)RegAddr << 16);
        CSR_WRITE_4(sc, RE_EPHYAR, TmpUlong);

        /* Wait for writing to Phy ok */
        for (Timeout=0; Timeout<5; Timeout++) {
                DELAY(1000);
                TmpUlong = CSR_READ_4(sc, RE_EPHYAR);
                if ((TmpUlong&PHYAR_Flag)!=0)
                        break;
        }

        RegData = (u_int16_t)(TmpUlong & 0x0000ffff);

        return RegData;
}

u_int16_t re_ephy_read(struct re_softc *sc, u_int8_t RegAddr)
{
        return _re_ephy_read(sc, RtCheckPciEPhyAddr(sc, RegAddr));
}

static void OOB_mutex_lock(struct re_softc *sc)
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

static void OOB_mutex_unlock(struct re_softc *sc)
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

void re_driver_start(struct re_softc *sc)
{
        if (!HW_DASH_SUPPORT_DASH(sc))
                return;

        if (!sc->AllowAccessDashOcp)
                return;

        re_notify_dash_oob(sc, OOB_CMD_DRIVER_START);

        re_wait_dash_fw_ready(sc);
}

void re_driver_stop(struct re_softc *sc)
{
        if (!HW_DASH_SUPPORT_DASH(sc))
                return;

        if (!sc->AllowAccessDashOcp)
                return;

        re_notify_dash_oob(sc, OOB_CMD_DRIVER_STOP);

        re_wait_dash_fw_ready(sc);
}

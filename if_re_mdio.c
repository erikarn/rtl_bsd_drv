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

#include "if_re_mdio.h"
#include "if_re_ocp.h"

void
re_mdio_write(struct re_softc *sc,u_int8_t RegAddr,u_int16_t RegData)
{
        u_int32_t		TmpUlong=0x80000000;
        u_int32_t		Timeout=0;

        if (RegAddr == 0x1F) {
                sc->cur_page = RegData;
        }

        if (sc->re_type == MACFG_63) {
                int i;
                CSR_WRITE_4(sc, RE_OCPDR, OCPDR_Write |
                            (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift |
                            (RegData & OCPDR_Data_Mask));
                CSR_WRITE_4(sc, RE_OCPAR, OCPAR_GPHY_Write);
                CSR_WRITE_4(sc, RE_EPHY_RXER_NUM, 0);

                for (i = 0; i < 100; i++) {
                        DELAY(1000);
                        if (!(CSR_READ_4(sc, RE_OCPAR) & OCPAR_Flag))
                                break;
                }
        } else if (sc->re_type == MACFG_56 || sc->re_type == MACFG_57 ||
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
                   sc->re_type == MACFG_91 || sc->re_type == MACFG_92) {
                if (RegAddr == 0x1F) {
                        return;
                }

                re_ocp_phy_write(sc, sc->cur_page, RegAddr, RegData);
        } else {
                if (sc->re_type == MACFG_65 || sc->re_type == MACFG_66)
                        CSR_WRITE_4(sc, 0xD0, CSR_READ_4(sc, 0xD0) & ~0x00020000);

                TmpUlong |= (((u_int32_t)RegAddr)<<16 | (u_int32_t)RegData);

                CSR_WRITE_4(sc, RE_PHYAR, TmpUlong);

                /* Wait for writing to Phy ok */
                for (Timeout=0; Timeout<5; Timeout++) {
                        DELAY(1000);
                        if ((CSR_READ_4(sc, RE_PHYAR)&PHYAR_Flag)==0)
                                break;
                }

                if (sc->re_type == MACFG_65 || sc->re_type == MACFG_66)
                        CSR_WRITE_4(sc, 0xD0, CSR_READ_4(sc, 0xD0) | 0x00020000);
        }
}

u_int16_t
re_mdio_read(struct re_softc *sc,u_int8_t RegAddr)
{
        u_int16_t		RegData;
        u_int32_t		TmpUlong;
        u_int32_t		Timeout=0;

        if (sc->re_type == MACFG_63) {
                int i;
                CSR_WRITE_4(sc, RE_OCPDR, OCPDR_Read |
                            (RegAddr & OCPDR_Reg_Mask) << OCPDR_GPHY_Reg_shift);
                CSR_WRITE_4(sc, RE_OCPAR, OCPAR_GPHY_Write);
                CSR_WRITE_4(sc, RE_EPHY_RXER_NUM, 0);

                for (i = 0; i < 100; i++) {
                        DELAY(1000);
                        if (!(CSR_READ_4(sc, RE_OCPAR) & OCPAR_Flag))
                                break;
                }

                DELAY(1000);
                CSR_WRITE_4(sc, RE_OCPAR, OCPAR_GPHY_Read);
                CSR_WRITE_4(sc, RE_EPHY_RXER_NUM, 0);

                for (i = 0; i < 100; i++) {
                        DELAY(1000);
                        if (CSR_READ_4(sc, RE_OCPAR) & OCPAR_Flag)
                                break;
                }

                RegData = CSR_READ_4(sc, RE_OCPDR) & OCPDR_Data_Mask;
        } else if (sc->re_type == MACFG_56 || sc->re_type == MACFG_57 ||
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
                   sc->re_type == MACFG_91 || sc->re_type == MACFG_92) {
                RegData = re_ocp_phy_read(sc, sc->cur_page, RegAddr);
        } else {
                if (sc->re_type == MACFG_65 || sc->re_type == MACFG_66)
                        CSR_WRITE_4(sc, 0xD0, CSR_READ_4(sc, 0xD0) & ~0x00020000);

                TmpUlong = ((u_int32_t)RegAddr << 16);
                CSR_WRITE_4(sc, RE_PHYAR, TmpUlong);

                /* Wait for writing to Phy ok */
                for (Timeout=0; Timeout<5; Timeout++) {
                        DELAY(1000);
                        TmpUlong = CSR_READ_4(sc, RE_PHYAR);
                        if ((TmpUlong&PHYAR_Flag)!=0)
                                break;
                }

                RegData = (u_int16_t)(TmpUlong & 0x0000ffff);

                if (sc->re_type == MACFG_65 || sc->re_type == MACFG_66)
                        CSR_WRITE_4(sc, 0xD0, CSR_READ_4(sc, 0xD0) | 0x00020000);
        }

        return RegData;
}


void
re_clear_set_eth_phy_bit(struct re_softc *sc, u_int8_t addr,
    u_int16_t clearmask, u_int16_t setmask)
{
	u_int16_t PhyRegValue;

	PhyRegValue = re_mdio_read(sc, addr);
	PhyRegValue &= ~clearmask;
	PhyRegValue |= setmask;
	re_mdio_write(sc, addr, PhyRegValue);
}

void
re_clear_eth_phy_bit(struct re_softc *sc, u_int8_t addr, u_int16_t mask)
{

	re_clear_set_eth_phy_bit(sc, addr, mask, 0);
}

void
re_set_eth_phy_bit(struct re_softc *sc, u_int8_t addr, u_int16_t mask)
{

	re_clear_set_eth_phy_bit(sc, addr, 0, mask);
}


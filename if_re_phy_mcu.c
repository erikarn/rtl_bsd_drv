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
#include "if_re_phy_mcu.h"

int
re_phy_ram_code_check(struct re_softc *sc)
{
        u_int16_t PhyRegValue;
        int retval = TRUE;

        switch(sc->re_type) {
        case MACFG_56:
                re_mdio_write(sc, 0x1f, 0x0A40);
                PhyRegValue = re_mdio_read(sc, 0x10);
                PhyRegValue &= ~(BIT_11);
                re_mdio_write(sc, 0x10, PhyRegValue);


                re_mdio_write(sc, 0x1f, 0x0A00);
                PhyRegValue = re_mdio_read(sc, 0x10);
                PhyRegValue &= ~(BIT_12 | BIT_13 | BIT_14 | BIT_15);
                re_mdio_write(sc, 0x10, PhyRegValue);

                re_mdio_write(sc, 0x1f, 0x0A43);
                re_mdio_write(sc, 0x13, 0x8010);
                PhyRegValue = re_mdio_read(sc, 0x14);
                PhyRegValue &= ~(BIT_11);
                re_mdio_write(sc, 0x14, PhyRegValue);

                re_set_phy_mcu_patch_request(sc);

                re_mdio_write(sc, 0x1f, 0x0A40);
                re_mdio_write(sc, 0x10, 0x0140);

                re_mdio_write(sc, 0x1f, 0x0A4A);
                PhyRegValue = re_mdio_read(sc, 0x13);
                PhyRegValue &= ~(BIT_6);
                PhyRegValue |= (BIT_7);
                re_mdio_write(sc, 0x13, PhyRegValue);

                re_mdio_write(sc, 0x1f, 0x0A44);
                PhyRegValue = re_mdio_read(sc, 0x14);
                PhyRegValue |= (BIT_2);
                re_mdio_write(sc, 0x14, PhyRegValue);

                re_mdio_write(sc, 0x1f, 0x0A50);
                PhyRegValue = re_mdio_read(sc, 0x11);
                PhyRegValue |= (BIT_11|BIT_12);
                re_mdio_write(sc, 0x11, PhyRegValue);

                re_clear_phy_mcu_patch_request(sc);

                re_mdio_write(sc, 0x1f, 0x0A40);
                re_mdio_write(sc, 0x10, 0x1040);

                re_mdio_write(sc, 0x1f, 0x0A4A);
                PhyRegValue = re_mdio_read(sc, 0x13);
                PhyRegValue &= ~(BIT_6|BIT_7);
                re_mdio_write(sc, 0x13, PhyRegValue);

                re_mdio_write(sc, 0x1f, 0x0A44);
                PhyRegValue = re_mdio_read(sc, 0x14);
                PhyRegValue &= ~(BIT_2);
                re_mdio_write(sc, 0x14, PhyRegValue);

                re_mdio_write(sc, 0x1f, 0x0A50);
                PhyRegValue = re_mdio_read(sc, 0x11);
                PhyRegValue &= ~(BIT_11|BIT_12);
                re_mdio_write(sc, 0x11, PhyRegValue);

                re_mdio_write(sc, 0x1f, 0x0A43);
                re_mdio_write(sc, 0x13, 0x8010);
                PhyRegValue = re_mdio_read(sc, 0x14);
                PhyRegValue |= (BIT_11);
                re_mdio_write(sc, 0x14, PhyRegValue);

                re_set_phy_mcu_patch_request(sc);

                re_mdio_write(sc, 0x1f, 0x0A20);
                PhyRegValue = re_mdio_read(sc, 0x13);
                if (PhyRegValue & BIT_11) {
                        if (PhyRegValue & BIT_10) {
                                retval = FALSE;
                        }
                }

                re_clear_phy_mcu_patch_request(sc);

                //delay 2ms
                DELAY(2000);
                break;
        default:
                break;
        }

        re_mdio_write(sc, 0x1F, 0x0000);

        return retval;
}

void
re_set_phy_ram_code_check_fail_flag(struct re_softc *sc)
{
        u_int16_t TmpUshort;

        switch(sc->re_type) {
        case MACFG_56:
                TmpUshort = re_mac_ocp_read(sc, 0xD3C0);
                TmpUshort |= BIT_0;
                re_mac_ocp_write(sc, 0xD3C0, TmpUshort);
                break;
        }
}

int
re_hw_phy_mcu_code_ver_matched(struct re_softc *sc)
{
        int ram_code_ver_match = 0;

        switch (sc->re_type) {
        case MACFG_36:
        case MACFG_37:
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B60);
                sc->re_hw_ram_code_ver = re_mdio_read(sc, 0x06);
                re_mdio_write(sc, 0x1F, 0x0000);
                break;
        case MACFG_38:
        case MACFG_39:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B30);
                sc->re_hw_ram_code_ver = re_mdio_read(sc, 0x06);
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
        case MACFG_76:
                re_mdio_write(sc, 0x1F, 0x0A43);
                re_mdio_write(sc, 0x13, 0x801E);
                sc->re_hw_ram_code_ver = re_mdio_read(sc, 0x14);
                re_mdio_write(sc, 0x1F, 0x0000);
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
                re_real_ocp_phy_write(sc, 0xA436, 0x801E);
                sc->re_hw_ram_code_ver = re_real_ocp_phy_read(sc, 0xA438);
                break;
        default:
                sc->re_hw_ram_code_ver = ~0;
                break;
        }

        if (sc->re_hw_ram_code_ver == sc->re_sw_ram_code_ver)
                ram_code_ver_match = 1;

        return ram_code_ver_match;
}

void
re_write_hw_phy_mcu_code_ver(struct re_softc *sc)
{
        switch (sc->re_type) {
        case MACFG_36:
        case MACFG_37:
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B60);
                re_mdio_write(sc, 0x06, sc->re_sw_ram_code_ver);
                re_mdio_write(sc, 0x1F, 0x0000);
                sc->re_hw_ram_code_ver = sc->re_sw_ram_code_ver;
                break;
        case MACFG_38:
        case MACFG_39:
        case MACFG_50:
        case MACFG_51:
        case MACFG_52:
                re_mdio_write(sc, 0x1F, 0x0005);
                re_mdio_write(sc, 0x05, 0x8B30);
                re_mdio_write(sc, 0x06, sc->re_sw_ram_code_ver);
                re_mdio_write(sc, 0x1F, 0x0000);
                sc->re_hw_ram_code_ver = sc->re_sw_ram_code_ver;
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
                re_mdio_write(sc, 0x1F, 0x0A43);
                re_mdio_write(sc, 0x13, 0x801E);
                re_mdio_write(sc, 0x14, sc->re_sw_ram_code_ver);
                re_mdio_write(sc, 0x1F, 0x0000);
                sc->re_hw_ram_code_ver = sc->re_sw_ram_code_ver;
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
                re_real_ocp_phy_write(sc, 0xA436, 0x801E);
                re_real_ocp_phy_write(sc, 0xA438, sc->re_sw_ram_code_ver);
                sc->re_hw_ram_code_ver = sc->re_sw_ram_code_ver;
                break;
        }
}

void
re_acquire_phy_mcu_patch_key_lock(struct re_softc *sc)
{
        u_int16_t PatchKey;

        switch (sc->re_type) {
        case MACFG_80:
                PatchKey = 0x8600;
                break;
        case MACFG_81:
                PatchKey = 0x8601;
                break;
        case MACFG_82:
                PatchKey = 0x3700;
                break;
        case MACFG_83:
                PatchKey = 0x3701;
                break;
        default:
                return;
        }
        re_real_ocp_phy_write(sc, 0xA436, 0x8024);
        re_real_ocp_phy_write(sc, 0xA438, PatchKey);
        re_real_ocp_phy_write(sc, 0xA436, 0xB82E);
        re_real_ocp_phy_write(sc, 0xA438, 0x0001);
}

void
re_release_phy_mcu_patch_key_lock(struct re_softc *sc)
{
        switch (sc->re_type) {
        case MACFG_80:
        case MACFG_81:
        case MACFG_82:
        case MACFG_83:
                re_real_ocp_phy_write(sc, 0xA436, 0x0000);
                re_real_ocp_phy_write(sc, 0xA438, 0x0000);
                re_clear_eth_ocp_phy_bit(sc, 0xB82E, BIT_0);
                re_real_ocp_phy_write(sc, 0xA436, 0x8024);
                re_real_ocp_phy_write(sc, 0xA438, 0x0000);
                break;
        default:
                break;
        }
}

bool
re_set_phy_mcu_patch_request(struct re_softc *sc)
{
        u_int16_t PhyRegValue;
        u_int16_t WaitCount = 0;
        bool bSuccess = TRUE;

        switch (sc->re_type) {
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
                re_mdio_write(sc, 0x1f, 0x0B82);
                re_set_eth_phy_bit(sc, 0x10, BIT_4);

                re_mdio_write(sc, 0x1f, 0x0B80);
                WaitCount = 0;
                do {
                        PhyRegValue = re_mdio_read(sc, 0x10);
                        DELAY(50);
                        DELAY(50);
                        WaitCount++;
                } while (!(PhyRegValue & BIT_6) && (WaitCount < 1000));

                if (!(PhyRegValue & BIT_6) && (WaitCount == 1000)) bSuccess = FALSE;

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
                re_set_eth_ocp_phy_bit(sc, 0xB820, BIT_4);

                WaitCount = 0;
                do {
                        PhyRegValue = re_real_ocp_phy_read(sc, 0xB800);
                        DELAY(50);
                        DELAY(50);
                        WaitCount++;
                } while (!(PhyRegValue & BIT_6) && (WaitCount < 1000));

                if (!(PhyRegValue & BIT_6) && (WaitCount == 1000)) bSuccess = FALSE;

                break;
        }

        return bSuccess;
}

bool
re_clear_phy_mcu_patch_request(struct re_softc *sc)
{
        u_int16_t PhyRegValue;
        u_int16_t WaitCount = 0;
        bool bSuccess = TRUE;

        switch (sc->re_type) {
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
                re_mdio_write(sc, 0x1f, 0x0B82);
                re_clear_eth_phy_bit(sc, 0x10, BIT_4);

                re_mdio_write(sc, 0x1f, 0x0B80);
                WaitCount = 0;
                do {
                        PhyRegValue = re_mdio_read(sc, 0x10);
                        DELAY(50);
                        DELAY(50);
                        WaitCount++;
                } while ((PhyRegValue & BIT_6) && (WaitCount < 1000));

                if ((PhyRegValue & BIT_6) && (WaitCount == 1000)) bSuccess = FALSE;

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
                re_clear_eth_ocp_phy_bit(sc, 0xB820, BIT_4);

                WaitCount = 0;
                do {
                        PhyRegValue = re_real_ocp_phy_read(sc, 0xB800);
                        DELAY(50);
                        DELAY(50);
                        WaitCount++;
                } while ((PhyRegValue & BIT_6) && (WaitCount < 1000));

                if ((PhyRegValue & BIT_6) && (WaitCount == 1000)) bSuccess = FALSE;

                break;
        }

        return bSuccess;
}

void
re_set_phy_mcu_ram_code(struct re_softc *sc, const u_int16_t *ramcode, u_int16_t codesize)
{
        u_int16_t i;
        u_int16_t addr;
        u_int16_t val;

        if (ramcode == NULL || codesize % 2) {
                goto out;
        }

        for (i = 0; i < codesize; i += 2) {
                addr = ramcode[i];
                val = ramcode[i + 1];
                if (addr == 0xFFFF && val == 0xFFFF) {
                        break;
                }
                re_real_ocp_phy_write(sc, addr, val);
        }

out:
        return;
}


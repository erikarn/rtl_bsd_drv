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

//#include "if_re_eeprom.h"
//#include "if_re_mdio.h"
#include "if_re_eri.h"
#include "if_re_ocp.h"
#include "if_re_cfg.h"
#include "if_re_mac_mcu.h"

static void
_re_enable_aspm_clkreq_lock(struct re_softc *sc, bool enable)
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

void
re_enable_aspm_clkreq_lock(struct re_softc *sc, bool enable)
{
        re_enable_cfg9346_write(sc);
        _re_enable_aspm_clkreq_lock(sc, enable);
        re_disable_cfg9346_write(sc);
}

void
re_disable_mcu_bps(struct re_softc *sc)
{
        u_int16_t regAddr;

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
                re_enable_aspm_clkreq_lock(sc, 0);
                break;
        }

        switch(sc->re_type) {
        case MACFG_68:
        case MACFG_69:
        case MACFG_70:
        case MACFG_71:
        case MACFG_72:
        case MACFG_73:
        case MACFG_74:
        case MACFG_75:
        case MACFG_76:
                re_mac_ocp_write(sc, 0xFC38, 0x0000);
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
                re_mac_ocp_write(sc, 0xFC48, 0x0000);
                break;
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
                re_mac_ocp_write(sc, 0xFC28, 0x0000);
                re_mac_ocp_write(sc, 0xFC2A, 0x0000);
                re_mac_ocp_write(sc, 0xFC2C, 0x0000);
                re_mac_ocp_write(sc, 0xFC2E, 0x0000);
                re_mac_ocp_write(sc, 0xFC30, 0x0000);
                re_mac_ocp_write(sc, 0xFC32, 0x0000);
                re_mac_ocp_write(sc, 0xFC34, 0x0000);
                re_mac_ocp_write(sc, 0xFC36, 0x0000);

                DELAY(3000);

                re_mac_ocp_write(sc, 0xFC26, 0x0000);
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
                for (regAddr = 0xFC28; regAddr < 0xFC48; regAddr += 2) {
                        re_mac_ocp_write(sc, regAddr, 0x0000);
                }

                DELAY(3000);

                re_mac_ocp_write(sc, 0xFC26, 0x0000);
                break;
        }
}

void
re_switch_mac_mcu_ram_code_page(struct re_softc *sc, u_int16_t page)
{
        u_int16_t tmpUshort;

        page &= (BIT_1 | BIT_0);
        tmpUshort = re_mac_ocp_read(sc, 0xE446);
        tmpUshort &= ~(BIT_1 | BIT_0);
        tmpUshort |= page;
        re_mac_ocp_write(sc, 0xE446, tmpUshort);
}

static void
_re_write_mac_mcu_ram_code(struct re_softc *sc, const u_int16_t *entry,
    u_int16_t entry_cnt)
{
        u_int16_t i;

        for (i = 0; i < entry_cnt; i++) {
                re_mac_ocp_write(sc, 0xF800 + i * 2, entry[i]);
        }
}

static void
_re_write_mac_mcu_ram_code_with_page(struct re_softc *sc,
    const u_int16_t *entry, u_int16_t entry_cnt, u_int16_t page_size)
{
        u_int16_t i;
        u_int16_t offset;

        if (page_size == 0) return;

        for (i = 0; i < entry_cnt; i++) {
                offset = i % page_size;
                if (offset == 0) {
                        u_int16_t page = (i / page_size);
                        re_switch_mac_mcu_ram_code_page(sc, page);
                }
                re_mac_ocp_write(sc, 0xF800 + offset * 2, entry[i]);
        }
}

void
re_write_mac_mcu_ram_code(struct re_softc *sc, const u_int16_t *entry,
    u_int16_t entry_cnt)
{
        if (FALSE == HW_SUPPORT_MAC_MCU(sc)) return;
        if (entry == NULL || entry_cnt == 0) return;

        if (sc->MacMcuPageSize > 0)
                _re_write_mac_mcu_ram_code_with_page(sc, entry, entry_cnt, sc->MacMcuPageSize);
        else
                _re_write_mac_mcu_ram_code(sc, entry, entry_cnt);
}


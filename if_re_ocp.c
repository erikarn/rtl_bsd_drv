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
#include "if_re_eri.h"

static u_int16_t
MappingPhyOcpAddress(struct re_softc *sc, u_int16_t PageNum, u_int8_t RegNum)
{
        u_int16_t OcpPageNum = 0;
        u_int8_t OcpRegNum = 0;
        u_int16_t OcpPhyAddress = 0;

        if (PageNum == 0) {
                OcpPageNum = 0x0A40 + (RegNum / 8);
                OcpRegNum = 0x10 + (RegNum % 8);
        } else {
                OcpPageNum = PageNum;
                OcpRegNum = RegNum;
        }

        OcpPageNum <<= 4;

        if (OcpRegNum < 16) {
                OcpPhyAddress = 0;
        } else {
                OcpRegNum -= 16;
                OcpRegNum <<= 1;

                OcpPhyAddress = OcpPageNum + OcpRegNum;
        }

        return OcpPhyAddress;
}

u_int16_t re_real_ocp_phy_read(
        struct re_softc *sc,
        u_int16_t OcpRegAddr)
{
        u_int32_t Timeout = 0, WaitCount = 100;
        u_int32_t TmpUlong;
        u_int16_t RetVal;

        TmpUlong = OcpRegAddr / 2;
        TmpUlong <<= 16;

        CSR_WRITE_4(sc, RE_PHYOCPACCESS, TmpUlong);

        do {
                DELAY(1);

                TmpUlong = CSR_READ_4(sc, RE_PHYOCPACCESS);

                Timeout++;
        } while ((!(TmpUlong & PHYAR_Flag)) && (Timeout < WaitCount));

        RetVal = (u_int16_t)TmpUlong;

        return RetVal;
}

u_int16_t re_ocp_phy_read(
        struct re_softc *sc,
        u_int16_t PhyPage,
        u_int8_t PhyRegNum)
{
        u_int16_t OcpRegAddr;
        u_int16_t RetVal;

        OcpRegAddr = MappingPhyOcpAddress(sc, PhyPage, PhyRegNum);

        if (OcpRegAddr % 2) {
                u_int16_t tmpUshort;

                tmpUshort = re_real_ocp_phy_read(sc, OcpRegAddr);
                tmpUshort &= 0xFF00;
                tmpUshort >>= 8;
                RetVal = tmpUshort;


                tmpUshort = re_real_ocp_phy_read(sc, OcpRegAddr + 1);
                tmpUshort &= 0x00FF;
                tmpUshort <<= 8;
                RetVal |= tmpUshort;
        } else {
                RetVal = re_real_ocp_phy_read(sc, OcpRegAddr);
        }

        return RetVal;
}

void re_real_ocp_phy_write(
        struct re_softc *sc,
        u_int16_t OcpRegAddr,
        u_int16_t RegData)
{
        u_int32_t Timeout = 0, WaitCount = 100;
        u_int32_t TmpUlong;

        TmpUlong = OcpRegAddr / 2;
        TmpUlong <<= 16;
        TmpUlong += RegData;
        TmpUlong |= BIT_31;

        CSR_WRITE_4(sc, RE_PHYOCPACCESS, TmpUlong);

        do {
                DELAY(1);

                TmpUlong = CSR_READ_4(sc, RE_PHYOCPACCESS);

                Timeout++;
        } while ((TmpUlong & PHYAR_Flag) && (Timeout < WaitCount));
}

void re_ocp_phy_write(
        struct re_softc *sc,
        u_int16_t PhyPage,
        u_int8_t PhyRegNum,
        u_int16_t RegData)
{
        u_int16_t OcpRegAddr;

        OcpRegAddr = MappingPhyOcpAddress(sc, PhyPage, PhyRegNum);

        if (OcpRegAddr % 2) {
                u_int16_t tmpUshort;

                tmpUshort = re_real_ocp_phy_read(sc, OcpRegAddr);
                tmpUshort &= 0x00FF;
                tmpUshort |= (RegData <<  8);
                re_real_ocp_phy_write(sc, OcpRegAddr, tmpUshort);
                tmpUshort = re_real_ocp_phy_read(sc, OcpRegAddr + 1);
                tmpUshort &= 0xFF00;
                tmpUshort |= (RegData >> 8);
                re_real_ocp_phy_write(sc, OcpRegAddr + 1, tmpUshort);
        } else {
                re_real_ocp_phy_write(sc, OcpRegAddr, RegData);
        }
}

void re_mac_ocp_write(
        struct re_softc *sc,
        u_int16_t ExtRegAddr,
        u_int16_t RegData)
{
        u_int32_t TmpUlong;

        TmpUlong = ExtRegAddr / 2;
        TmpUlong <<= 16;
        TmpUlong += RegData;
        TmpUlong |= BIT_31;

        CSR_WRITE_4(sc, RE_MCUACCESS, TmpUlong);
}

u_int16_t re_mac_ocp_read(
        struct re_softc *sc,
        u_int16_t ExtRegAddr)
{
        u_int32_t TmpUlong;
        u_int16_t RetVal;

        TmpUlong = ExtRegAddr / 2;
        TmpUlong <<= 16;

        CSR_WRITE_4(sc, RE_MCUACCESS, TmpUlong);
        TmpUlong = CSR_READ_4(sc, RE_MCUACCESS);
        RetVal = (u_int16_t)TmpUlong;

        return RetVal;
}

u_int32_t real_ocp_read(struct re_softc *sc, u_int16_t addr, u_int8_t len)
{
        int i, val_shift, shift = 0;
        u_int32_t value1 = 0, value2 = 0, mask;

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % 4;
                addr = addr & ~0x3;

                CSR_WRITE_4(sc, RE_OCPAR, (0x0F<<12) | (addr&0xFFF));

                for (i = 0; i < 20; i++) {
                        DELAY(100);
                        if (CSR_READ_4(sc, RE_OCPAR) & OCPAR_Flag)
                                break;
                }

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = CSR_READ_4(sc, RE_OCPDR) & mask;
                value2 |= (value1 >> val_shift * 8) << shift * 8;

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        DELAY(20);

        return value2;
}

u_int32_t
re_ocp_read_with_oob_base_address(struct re_softc *sc, u_int16_t addr,
    u_int8_t len, const u_int32_t base_address)
{
        return re_eri_read_with_oob_base_address(sc, addr, len, ERIAR_OOB, base_address);
}

u_int32_t
re_ocp_read(struct re_softc *sc, u_int16_t addr, u_int8_t len)
{
        u_int32_t value = 0;

        if (!sc->AllowAccessDashOcp)
                return 0xffffffff;

        if (sc->HwSuppOcpChannelVer == 1)
                value = real_ocp_read(sc, addr, len);
        else if (sc->HwSuppOcpChannelVer == 2)
                value = re_eri_read(sc, addr, len, ERIAR_OOB);
        else if (sc->HwSuppOcpChannelVer == 3)
                value = re_ocp_read_with_oob_base_address(sc, addr, len, RTL8168FP_OOBMAC_BASE);

        return value;
}

int
real_ocp_write(struct re_softc *sc, u_int16_t addr, u_int8_t len,
    u_int32_t value)
{
        int i, val_shift, shift = 0;
        u_int32_t value1 = 0, mask;

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % 4;
                addr = addr & ~0x3;

                if (len == 1)       mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)  mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)  mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else            mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = re_ocp_read(sc, addr, 4) & ~mask;
                value1 |= ((value << val_shift * 8) >> shift * 8);

                CSR_WRITE_4(sc, RE_OCPDR, value1);
                CSR_WRITE_4(sc, RE_OCPAR, OCPAR_Flag | (0x0F<<12) | (addr&0xFFF));

                for (i = 0; i < 10; i++) {
                        DELAY(100);

                        /* Check if the RTL8168 has completed ERI write */
                        if (!(CSR_READ_4(sc, RE_OCPAR) & OCPAR_Flag))
                                break;
                }

                if (len <= 4 - val_shift) {
                        len = 0;
                } else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        DELAY(20);

        return 0;
}

int
re_ocp_write_with_oob_base_address(struct re_softc *sc, u_int16_t addr,
    u_int8_t len, u_int32_t value, const u_int32_t base_address)
{
        return re_eri_write_with_oob_base_address(sc, addr, len, value, ERIAR_OOB, base_address);
}

void
re_ocp_write(struct re_softc *sc, u_int16_t addr, u_int8_t len, u_int32_t value)
{
        if (!sc->AllowAccessDashOcp)
                return;

        if (sc->HwSuppOcpChannelVer == 1)
                real_ocp_write(sc, addr, len, value);
        else if (sc->HwSuppOcpChannelVer == 2)
                re_eri_write(sc, addr, len, value, ERIAR_OOB);
        else if (sc->HwSuppOcpChannelVer == 3)
                re_ocp_write_with_oob_base_address(sc, addr, len, value, RTL8168FP_OOBMAC_BASE);
}

void
re_clear_set_eth_ocp_phy_bit(struct re_softc *sc, u_int16_t addr,
    u_int16_t clearmask, u_int16_t setmask)
{
	u_int16_t PhyRegValue;

	PhyRegValue = re_real_ocp_phy_read(sc, addr);
	PhyRegValue &= ~clearmask;
	PhyRegValue |= setmask;
	re_real_ocp_phy_write(sc, addr, PhyRegValue);
}

void
re_clear_eth_ocp_phy_bit( struct re_softc *sc, u_int16_t addr,
    u_int16_t mask)
{

	re_clear_set_eth_ocp_phy_bit(sc, addr, mask, 0);
}

void
re_set_eth_ocp_phy_bit(struct re_softc *sc, u_int16_t addr,
    u_int16_t mask)
{

	re_clear_set_eth_ocp_phy_bit(sc, addr, 0, mask);
}

void
re_clear_set_mac_ocp_bit(struct re_softc *sc, u_int16_t addr,
    u_int16_t clearmask, u_int16_t setmask)
{
	u_int16_t PhyRegValue;

	PhyRegValue = re_mac_ocp_read(sc, addr);
	PhyRegValue &= ~clearmask;
	PhyRegValue |= setmask;
	re_mac_ocp_write(sc, addr, PhyRegValue);
}

void
re_clear_mac_ocp_bit(struct re_softc *sc, u_int16_t addr,
    u_int16_t mask)
{

	re_clear_set_mac_ocp_bit(sc, addr, mask, 0);
}

void
re_set_mac_ocp_bit(struct re_softc *sc, u_int16_t addr,
    u_int16_t mask)
{

	re_clear_set_mac_ocp_bit(sc, addr, 0, mask);
}

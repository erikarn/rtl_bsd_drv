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

#include "if_re_efuse.h"

static u_int8_t
re_calc_efuse_dummy_bit(u_int16_t reg)
{
        int s,a;
        u_int8_t dummyBitPos = 0;


        s=reg% 32;
        a=s % 16;

        if (s/16) {
                dummyBitPos = (u_int8_t)(16-a);
        } else {
                dummyBitPos = (u_int8_t)a;
        }

        return dummyBitPos;
}

u_int32_t
re_decode_efuse_cmd(struct re_softc *sc, u_int32_t DwCmd)
{
        u_int16_t reg = (u_int16_t)((DwCmd & 0x00FE0000) >> 17);
        u_int32_t DummyPos = re_calc_efuse_dummy_bit(reg);
        u_int32_t DeCodeDwCmd;
        u_int32_t Dw17BitData;


        if (sc->re_efuse_ver < 3) {
                DeCodeDwCmd = (DwCmd>>(DummyPos+1))<<DummyPos;
                if (DummyPos > 0)
                        DeCodeDwCmd |= ((DwCmd<<(32-DummyPos))>>(32-DummyPos));
        } else {
                reg = (u_int16_t)((DwCmd & 0x007F0000) >> 16);
                DummyPos = re_calc_efuse_dummy_bit(reg);
                Dw17BitData = ((DwCmd & BIT_23) >> 23);
                Dw17BitData <<= 16;
                Dw17BitData |= (DwCmd & 0x0000FFFF);
                DeCodeDwCmd = (Dw17BitData>>(DummyPos+1))<<DummyPos;
                if (DummyPos > 0)
                        DeCodeDwCmd |= ((Dw17BitData<<(32-DummyPos))>>(32-DummyPos));
        }

        return DeCodeDwCmd;
}

#define EFUSE_WRITE 0x80000000
#define EFUSE_WRITE_OK  0x00000000
#define EFUSE_READ  0x00000000
#define EFUSE_READ_OK  0x80000000
#define EFUSE_Reg_Mask 0x03FF
#define EFUSE_Reg_Shift 8
#define EFUSE_Check_Cnt 300
#define EFUSE_READ_FAIL 0xFF
#define EFUSE_Data_Mask 0x000000FF

u_int8_t
re_efuse_read(struct re_softc *sc, u_int16_t reg)
{
        u_int8_t efuse_data = 0;
        u_int32_t temp;
        u_int32_t cnt;

        if (sc->re_efuse_ver == EFUSE_NOT_SUPPORT)
                return EFUSE_READ_FAIL;

        if (sc->re_efuse_ver == EFUSE_SUPPORT_V1) {
                temp = EFUSE_READ | ((reg & EFUSE_Reg_Mask) << EFUSE_Reg_Shift);
                CSR_WRITE_4(sc, RE_EFUSEAR, temp);

                cnt = 0;
                do {
                        DELAY(100);
                        temp = CSR_READ_4(sc, RE_EFUSEAR);
                        cnt++;
                } while (!(temp & EFUSE_READ_OK) && (cnt < EFUSE_Check_Cnt));

                if (cnt == EFUSE_Check_Cnt)
                        efuse_data = EFUSE_READ_FAIL;
                else
                        efuse_data = (u_int8_t)(CSR_READ_4(sc, RE_EFUSEAR) & EFUSE_Data_Mask);
        } else  if (sc->re_efuse_ver == EFUSE_SUPPORT_V2) {
                temp = (reg/2) & 0x03ff;
                temp <<= 17;
                temp |= EFUSE_READ;
                CSR_WRITE_4(sc, RE_EFUSEAR, temp);

                cnt = 0;
                do {
                        DELAY(100);
                        temp = CSR_READ_4(sc, RE_EFUSEAR);
                        cnt++;
                } while (!(temp & EFUSE_READ_OK) && (cnt < EFUSE_Check_Cnt));

                if (cnt == EFUSE_Check_Cnt) {
                        efuse_data = EFUSE_READ_FAIL;
                } else {
                        temp = CSR_READ_4(sc, RE_EFUSEAR);
                        temp = re_decode_efuse_cmd(sc, temp);

                        if (reg%2) {
                                temp >>= 8;
                                efuse_data = (u_int8_t)temp;
                        } else {
                                efuse_data = (u_int8_t)temp;
                        }
                }
        } else  if (sc->re_efuse_ver == EFUSE_SUPPORT_V3) {
                temp = (reg/2) & 0x03ff;
                temp <<= 16;
                temp |= EFUSE_READ;
                CSR_WRITE_4(sc, RE_EFUSEAR, temp);

                cnt = 0;
                do {
                        DELAY(100);
                        temp = CSR_READ_4(sc, RE_EFUSEAR);
                        cnt++;
                } while (!(temp & EFUSE_READ_OK) && (cnt < EFUSE_Check_Cnt));

                if (cnt == EFUSE_Check_Cnt) {
                        efuse_data = EFUSE_READ_FAIL;
                } else {
                        temp = CSR_READ_4(sc, RE_EFUSEAR);
                        temp = re_decode_efuse_cmd(sc, temp);

                        if (reg%2) {
                                temp >>= 8;
                                efuse_data = (u_int8_t)temp;
                        } else {
                                efuse_data = (u_int8_t)temp;
                        }
                }
        }

        DELAY(20);

        return efuse_data;
}

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

#include "if_re_eri.h"

u_int32_t
re_eri_read_with_oob_base_address(struct re_softc *sc, int addr, int len,
    int type, const u_int32_t base_address)
{
        int i, val_shift, shift = 0;
        u_int32_t value1 = 0, value2 = 0, mask;
        const u_int32_t transformed_base_address = ((base_address & 0x00FFF000) << 6) | (base_address & 0x000FFF);

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % ERIAR_Addr_Align;
                addr = addr & ~0x3;

                CSR_WRITE_4(sc,RE_ERIAR,
                            ERIAR_Read |
                            transformed_base_address |
                            type << ERIAR_Type_shift |
                            ERIAR_ByteEn << ERIAR_ByteEn_shift |
                            addr);

                for (i = 0; i < 10; i++) {
                        DELAY(100);

                        /* Check if the RTL8168 has completed ERI read */
                        if (CSR_READ_4(sc,RE_ERIAR) & ERIAR_Flag)
                                break;
                }

                if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = CSR_READ_4(sc,RE_ERIDR) & mask;
                value2 |= (value1 >> val_shift * 8) << shift * 8;

                if (len <= 4 - val_shift)
                        len = 0;
                else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        return value2;
}

u_int32_t
re_eri_read(struct re_softc *sc, int addr, int len, int type)
{
        return re_eri_read_with_oob_base_address(sc, addr, len, type, 0);
}

int
re_eri_write_with_oob_base_address(struct re_softc *sc, int addr, int len,
    u_int32_t value, int type, const u_int32_t base_address)
{
        int i, val_shift, shift = 0;
        u_int32_t value1 = 0, mask;
        const u_int32_t transformed_base_address = ((base_address & 0x00FFF000) << 6) | (base_address & 0x000FFF);

        if (len > 4 || len <= 0)
                return -1;

        while (len > 0) {
                val_shift = addr % ERIAR_Addr_Align;
                addr = addr & ~0x3;

                if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
                else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;

                value1 = re_eri_read_with_oob_base_address(sc, addr, 4, type, base_address) & ~mask;
                value1 |= ((value << val_shift * 8) >> shift * 8);

                CSR_WRITE_4(sc,RE_ERIDR, value1);
                CSR_WRITE_4(sc,RE_ERIAR,
                            ERIAR_Write |
                            transformed_base_address |
                            type << ERIAR_Type_shift |
                            ERIAR_ByteEn << ERIAR_ByteEn_shift |
                            addr);

                for (i = 0; i < 10; i++) {
                        DELAY(100);

                        /* Check if the RTL8168 has completed ERI write */
                        if (!(CSR_READ_4(sc,RE_ERIAR) & ERIAR_Flag))
                                break;
                }

                if (len <= 4 - val_shift)
                        len = 0;
                else {
                        len -= (4 - val_shift);
                        shift = 4 - val_shift;
                        addr += 4;
                }
        }

        return 0;
}

int
re_eri_write(struct re_softc *sc, int addr, int len, u_int32_t value, int type)
{
        return re_eri_write_with_oob_base_address(sc, addr, len, value, type, 0);
}

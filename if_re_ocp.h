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
#ifndef	__IF_RE_OCP_H__
#define	__IF_RE_OCP_H__

struct re_softc;

u_int16_t re_real_ocp_phy_read( struct re_softc *sc, u_int16_t OcpRegAddr);
u_int16_t re_ocp_phy_read( struct re_softc *sc, u_int16_t PhyPage, u_int8_t PhyRegNum);
void re_real_ocp_phy_write( struct re_softc *sc, u_int16_t OcpRegAddr, u_int16_t RegData);
void re_ocp_phy_write( struct re_softc *sc, u_int16_t PhyPage, u_int8_t PhyRegNum, u_int16_t RegData);
void re_mac_ocp_write( struct re_softc *sc, u_int16_t ExtRegAddr, u_int16_t RegData);
u_int16_t re_mac_ocp_read( struct re_softc *sc, u_int16_t ExtRegAddr);
u_int32_t real_ocp_read(struct re_softc *sc, u_int16_t addr, u_int8_t len);
u_int32_t re_ocp_read_with_oob_base_address(struct re_softc *sc,
    u_int16_t addr, u_int8_t len, const u_int32_t base_address);
u_int32_t re_ocp_read(struct re_softc *sc, u_int16_t addr, u_int8_t len);
int real_ocp_write(struct re_softc *sc, u_int16_t addr, u_int8_t len, u_int32_t value);
int re_ocp_write_with_oob_base_address(struct re_softc *sc, u_int16_t addr,
    u_int8_t len, u_int32_t value, const u_int32_t base_address);
void re_ocp_write(struct re_softc *sc, u_int16_t addr, u_int8_t len, u_int32_t value);

#endif	/* __IF_RE_OCP_H__ */

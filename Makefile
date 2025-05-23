# $FreeBSD: src/sys/modules/re/Makefile,v 1.6 2000/01/28 11:26:34 bde Exp $

enable_fiber_support = n
enable_s5wol = n
enable_eee = n
enable_s0_magic_packet = n
config_soc_lan = n
interrupt_mitigation = y

.PATH:	${.CURDIR}/../../dev/re
KMOD	= if_re
SRCS	= if_re.c if_re_eeprom.c if_re_mdio.c if_re_ocp.c if_re_eri.c \
	  if_re_csi.c \
	  if_re_cfg.c \
	  if_re_dash.c \
	  if_re_efuse.c \
	  if_re_misc.c \
	  if_re_mac_mcu.c \
	  if_re_phy_mcu.c \
	  if_re_oob_mutex.c \
	  chipset/8125/if_re_mac_8125.c \
	  chipset/8125/if_re_phy_8125.c \
	  chipset/8125/if_re_hw_8125.c \
	  chipset/8126/if_re_mac_8126.c \
	  chipset/8126/if_re_phy_8126.c \
	  chipset/8168/if_re_mac_8168.c \
	  chipset/8168/if_re_phy_8168.c \
	  chipset/8169/if_re_phy_8169.c \
	  chipset/8411/if_re_mac_8411.c \
	  chipset/8411/if_re_phy_8411.c

SRCS+=	  chipset/legacy/if_re_phy_macfg6.c \
	  chipset/legacy/if_re_phy_macfg14.c \
	  chipset/legacy/if_re_phy_macfg15.c \
	  chipset/legacy/if_re_phy_macfg17.c \
	  chipset/legacy/if_re_phy_macfg21.c \
	  chipset/legacy/if_re_phy_macfg22.c \
	  chipset/legacy/if_re_phy_macfg23.c \
	  chipset/legacy/if_re_phy_macfg24.c \
	  chipset/legacy/if_re_phy_macfg25.c \
	  chipset/legacy/if_re_phy_macfg26.c \
	  chipset/legacy/if_re_phy_macfg27.c \
	  chipset/legacy/if_re_phy_macfg28.c \
	  chipset/legacy/if_re_phy_macfg31.c \
	  chipset/legacy/if_re_phy_macfg32.c \
	  chipset/legacy/if_re_phy_macfg33.c \
	  chipset/legacy/if_re_phy_macfg36.c \
	  chipset/legacy/if_re_phy_macfg38.c \
	  chipset/legacy/if_re_phy_macfg39.c \
	  chipset/legacy/if_re_phy_macfg41.c \
	  chipset/legacy/if_re_phy_macfg42.c \
	  chipset/legacy/if_re_phy_macfg50.c \
	  chipset/legacy/if_re_phy_macfg51.c \
	  chipset/legacy/if_re_phy_macfg52.c \
	  chipset/legacy/if_re_phy_macfg53.c \
	  chipset/legacy/if_re_phy_macfg54.c \
	  chipset/legacy/if_re_phy_macfg56.c \
	  chipset/legacy/if_re_phy_macfg58.c \
	  chipset/legacy/if_re_phy_macfg59.c \
	  chipset/legacy/if_re_phy_macfg60.c \
	  chipset/legacy/if_re_phy_macfg61.c \
	  chipset/legacy/if_re_phy_macfg62.c \
	  chipset/legacy/if_re_phy_macfg63.c \
	  chipset/legacy/if_re_phy_macfg64.c \
	  chipset/legacy/if_re_phy_macfg65.c \
	  chipset/legacy/if_re_phy_macfg66.c \
	  chipset/legacy/if_re_phy_macfg68.c \
	  chipset/legacy/if_re_phy_macfg80.c \
	  chipset/legacy/if_re_phy_macfg82.c \
	  chipset/legacy/if_re_phy_macfg90.c \
	  opt_bdg.h device_if.h bus_if.h pci_if.h opt_inet.h opt_inet6.h

.if $(enable_fiber_support) == y
SRCS	+= if_fiber.c
CFLAGS	+= -DENABLE_FIBER_SUPPORT
.endif

.if $(enable_s5wol) == y
CFLAGS	+= -DENABLE_S5WOL
.endif

.if $(enable_eee) == y
CFLAGS	+= -DENABLE_EEE
.endif

.if $(enable_s0_magic_packet) == y
CFLAGS	+= -DENABLE_S0_MAGIC_PACKET
.endif

.if $(config_soc_lan) == y
CFLAGS	+= -DCONFIG_SOC_LAN
.endif

.if $(interrupt_mitigation) == y
CFLAGS	+= -DENABLE_INTERRUPT_MITIGATIN
.endif

.include <bsd.kmod.mk>

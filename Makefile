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
	  if_re_mac_8125.c \
	  if_re_phy_8125.c \
	  if_re_hw_8125.c \
	  if_re_mac_8126.c \
	  if_re_phy_8126.c \
	  if_re_mac_8168.c \
	  if_re_phy_8168.c \
	  if_re_phy_8169.c \
	  if_re_mac_8411.c \
	  if_re_phy_8411.c \
	  if_re_phy_macfg24.c \
	  if_re_phy_macfg25.c \
	  if_re_phy_macfg26.c \
	  if_re_phy_macfg27.c \
	  if_re_phy_macfg28.c \
	  if_re_phy_macfg31.c \
	  if_re_phy_macfg32.c \
	  if_re_phy_macfg33.c \
	  if_re_phy_macfg36.c \
	  if_re_phy_macfg38.c \
	  if_re_phy_macfg39.c \
	  if_re_phy_macfg41.c \
	  if_re_phy_macfg42.c \
	  if_re_phy_macfg50.c \
	  if_re_phy_macfg51.c \
	  if_re_phy_macfg52.c \
	  if_re_phy_macfg53.c \
	  if_re_phy_macfg54.c \
	  if_re_phy_macfg56.c \
	  if_re_phy_macfg58.c \
	  if_re_phy_macfg59.c \
	  if_re_phy_macfg60.c \
	  if_re_phy_macfg61.c \
	  if_re_phy_macfg62.c \
	  if_re_phy_macfg63.c \
	  if_re_phy_macfg64.c \
	  if_re_phy_macfg65.c \
	  if_re_phy_macfg66.c \
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

#
# SPDX-License-Identifier:	GPL-2.0+
#

dtb-$(CONFIG_TARGET_AP121) += ap121.dtb
dtb-$(CONFIG_TARGET_AP143) += ap143.dtb
dtb-$(CONFIG_TARGET_MALTA) += mti,malta.dtb
dtb-$(CONFIG_TARGET_PIC32MZDASK) += pic32mzda_sk.dtb
dtb-$(CONFIG_BOARD_TPLINK_WDR4300) += tplink_wdr4300.dtb

dtb-$(CONFIG_TARGET_EVB_GSC329X) += evb_gsc329x.dtb
dtb-$(CONFIG_TARGET_EVB_GSC32AX) += evb_gsc32ax.dtb

targets += $(dtb-y)

# Add any required device tree compiler flags here
DTC_FLAGS +=

PHONY += dtbs
dtbs: $(addprefix $(obj)/, $(dtb-y))
	@:

clean-files := *.dtb
